#include "xhci_priv.h"
#include <mm/kalloc.h>
#include <string.h>

#define XHCI_IN_CTRL            0
#define XHCI_IN_SLOT            1
#define XHCI_IN_EP(dci)         ((dci) + 1)
#define XHCI_OUT_SLOT           0
#define XHCI_OUT_EP(dci)        (dci)

#define XHCI_ADD_SLOT           (1u << 0)
#define XHCI_ADD_EP(dci)        (1u << (dci))

static void xhci_slot_free(xhci_slot_t *slot) {
    xhci_hc_t *hc = slot->hc;

    // out of the table under the event lock first, after that no event can reach it
    if (slot->id) {
        unsigned long irq = irq_push();
        spinlock_acquire(&hc->event_lock);
        hc->slots[slot->id] = NULL;
        spinlock_release(&hc->event_lock);
        irq_restore(irq);

        // the controller lets go of the output context here, only then can the DCBAA forget it
        xhci_cmd_disable_slot(hc, slot->id);
        hc->dcbaa[slot->id] = 0;
    }

    for (int dci = 0; dci < XHCI_MAX_DCI; dci++) {
        xhci_ep_t *ep = &slot->eps[dci];
        xhci_ring_free(&ep->ring);
        if (ep->buf)
            pmm_free_pages(ep->buf_phys, 1);
    }
    if (slot->in_ctx)   pmm_free_pages(slot->in_ctx_phys, 1);
    if (slot->out_ctx)  pmm_free_pages(slot->out_ctx_phys, 1);
    if (slot->ctrl_buf) pmm_free_pages(slot->ctrl_buf_phys, 1);
    kfree(slot);
}

void xhci_device_detach(xhci_hc_t *hc, xhci_slot_t *slot) {
    serial_printf(LOG_INFO "xhci%d: port %u slot %u going away\n", hc->index, slot->port, slot->id);

    // class drivers let go before anything they might still be using is freed
    usb_device_disconnect(&slot->dev);
    if (!slot->dev.parent)
        hc->port_slot[slot->port] = 0;
    xhci_slot_free(slot);
}

void xhci_device_remove(usb_device_t *dev) {
    xhci_device_detach(dev->hc, dev->hc_priv);
}

// what ep0 can take before we have read the device descriptor (4.3.3)
static uint16_t xhci_default_max_packet0(uint8_t speed) {
    switch (speed) {
        case XHCI_SPEED_LOW:   return 8;
        case XHCI_SPEED_FULL:
        case XHCI_SPEED_HIGH:  return 64;
        default:               return 512;
    }
}

static void xhci_fill_ep0(xhci_slot_t *slot, uint16_t max_packet) {
    uint32_t *ep = xhci_in_ctx(slot, XHCI_IN_EP(1));
    memset(ep, 0, slot->hc->ctx_size);

    ep[1] = XHCI_EP_CERR(3) | XHCI_EP_TYPE(XHCI_EP_TYPE_CONTROL) | XHCI_EP_MPS(max_packet);
    ep[2] = (uint32_t)(slot->eps[1].ring.phys | slot->eps[1].ring.cycle);
    ep[3] = (uint32_t)((uint64_t)slot->eps[1].ring.phys >> 32);
    ep[4] = XHCI_EP_AVG_TRB(8);     // control transfers are mostly the 8 byte setup packet
}

static xhci_slot_t *xhci_slot_alloc(xhci_hc_t *hc, uint8_t id, uint8_t port, uint8_t speed) {
    xhci_slot_t *slot = kmalloc(sizeof(*slot));
    if (!slot)
        return NULL;
    memset(slot, 0, sizeof(*slot));
    slot->hc    = hc;
    slot->port  = port;
    slot->speed = speed;

    bool low = !hc->ac64;
    slot->in_ctx   = usb_dma_alloc(1, &slot->in_ctx_phys, low);
    slot->out_ctx  = usb_dma_alloc(1, &slot->out_ctx_phys, low);
    slot->ctrl_buf = usb_dma_alloc(1, &slot->ctrl_buf_phys, low);

    // id goes in first so xhci_slot_free still disables the slot if anything below fails
    slot->id = id;
    if (!slot->in_ctx || !slot->out_ctx || !slot->ctrl_buf ||
        !xhci_ring_init(hc, &slot->eps[1].ring)) {
        serial_printf(LOG_ERROR "xhci%d: OOM setting up slot %u\n", hc->index, id);
        xhci_slot_free(slot);
        return NULL;
    }

    xhci_ep_t *ep0 = &slot->eps[1];
    ep0->used = true;
    ep0->dci  = 1;
    ep0->type = XHCI_EP_TYPE_CONTROL;

    hc->slots[id]  = slot;
    hc->dcbaa[id]  = slot->out_ctx_phys;
    return slot;
}

// the controller does SET_ADDRESS for us as part of Address Device (4.3.4)
static bool xhci_slot_address(xhci_slot_t *slot) {
    xhci_hc_t *hc = slot->hc;

    uint32_t *ctrl = xhci_in_ctx(slot, XHCI_IN_CTRL);
    ctrl[0] = 0;
    ctrl[1] = XHCI_ADD_SLOT | XHCI_ADD_EP(1);

    uint32_t *sctx = xhci_in_ctx(slot, XHCI_IN_SLOT);
    memset(sctx, 0, hc->ctx_size);
    slot->context_entries = 1;
    sctx[0] = XHCI_SLOT_ROUTE(slot->route) | XHCI_SLOT_SPEED(slot->speed) | XHCI_SLOT_ENTRIES(1);
    sctx[1] = XHCI_SLOT_ROOT_PORT(slot->port);
    sctx[2] = XHCI_SLOT_TT_HUB(slot->tt_slot) | XHCI_SLOT_TT_PORT(slot->tt_port);

    slot->eps[1].max_packet = xhci_default_max_packet0(slot->speed);
    xhci_fill_ep0(slot, slot->eps[1].max_packet);

    if (xhci_cmd_address_device(hc, slot) != XHCI_CC_SUCCESS)
        return false;

    uint32_t *out = xhci_out_ctx(slot, XHCI_OUT_SLOT);
    slot->dev.address = XHCI_SLOT_GET_ADDRESS(out[3]);
    serial_printf(LOG_INFO "xhci%d: slot %u port %u is at usb address %u\n",
                  hc->index, slot->id, slot->port, slot->dev.address);
    return true;
}

// ep0 might take more or less than the guess, the first 8 bytes of the device descriptor say
static bool xhci_slot_fix_max_packet0(xhci_slot_t *slot) {
    usb_device_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    if (usb_get_descriptor(&slot->dev, USB_DESC_DEVICE, &desc, 8) < 8) {
        serial_printf(LOG_WARN "xhci%d: port %u device isnt answering\n", slot->hc->index, slot->port);
        return false;
    }

    // usb3 gives it as a power of two
    uint16_t mps = slot->speed >= XHCI_SPEED_SUPER ? (uint16_t)(1u << desc.max_packet0)
                                                    : desc.max_packet0;
    if (mps < 8) {
        serial_printf(LOG_WARN "xhci%d: port %u says ep0 takes %u bytes, ignoring that\n",
                      slot->hc->index, slot->port, mps);
        return true;
    }
    if (mps == slot->eps[1].max_packet)
        return true;

    serial_printf(LOG_INFO "xhci%d: slot %u ep0 max packet %u -> %u\n",
                  slot->hc->index, slot->id, slot->eps[1].max_packet, mps);

    uint32_t *ctrl = xhci_in_ctx(slot, XHCI_IN_CTRL);
    ctrl[0] = 0;
    ctrl[1] = XHCI_ADD_EP(1);

    uint32_t *ep = xhci_in_ctx(slot, XHCI_IN_EP(1));
    ep[1] = (ep[1] & 0xFFFF) | XHCI_EP_MPS(mps);

    if (xhci_cmd_evaluate_ctx(slot->hc, slot) != XHCI_CC_SUCCESS)
        return false;
    slot->eps[1].max_packet = mps;
    slot->dev.max_packet0   = mps;
    return true;
}

// each hub tier gets a nibble, ports past 15 share the last value (8.9)
static void xhci_slot_place(xhci_slot_t *slot, usb_device_t *parent, uint8_t port) {
    xhci_slot_t *hub  = parent->hc_priv;
    uint32_t     tier = port > 15 ? 15 : port;

    slot->route = hub->route | (tier << (parent->depth * 4));

    if (slot->speed != XHCI_SPEED_LOW && slot->speed != XHCI_SPEED_FULL)
        return;
    if (parent->speed == USB_SPEED_HIGH) {
        slot->tt_slot = hub->id;
        slot->tt_port = port;
    } else {
        slot->tt_slot = hub->tt_slot;
        slot->tt_port = hub->tt_port;
    }
}

static xhci_slot_t *xhci_device_enumerate(xhci_hc_t *hc, uint8_t root_port, uint8_t speed,
                                          usb_device_t *parent, uint8_t port) {
    uint8_t id = 0;
    if (xhci_cmd_enable_slot(hc, &id) != XHCI_CC_SUCCESS || id == 0)
        return NULL;
    if (id > XHCI_MAX_SLOTS) {
        serial_printf(LOG_WARN "xhci%d: got slot %u, we only track %u\n", hc->index, id, XHCI_MAX_SLOTS);
        xhci_cmd_disable_slot(hc, id);
        return NULL;
    }

    xhci_slot_t *slot = xhci_slot_alloc(hc, id, root_port, speed);
    if (!slot)
        return NULL;
    if (parent)
        xhci_slot_place(slot, parent, port);

    usb_device_t *dev = &slot->dev;
    dev->ops         = &xhci_ops;
    dev->hc          = hc;
    dev->hc_priv     = slot;
    dev->port        = parent ? port : root_port;
    dev->parent      = parent;
    dev->depth       = parent ? parent->depth + 1 : 0;
    dev->speed       = xhci_usb_speed(speed);
    dev->max_packet0 = xhci_default_max_packet0(speed);

    serial_printf(LOG_INFO "xhci%d: port %u %s speed device on slot %u, route 0x%05x tt %u:%u\n",
                  hc->index, root_port, usb_speed_name(dev->speed), id, slot->route,
                  slot->tt_slot, slot->tt_port);

    if (!xhci_slot_address(slot)) {
        serial_printf(LOG_WARN "xhci%d: port %u wont take an address\n", hc->index, root_port);
        xhci_slot_free(slot);
        return NULL;
    }
    usb_delay_ms(2);   // 2ms to start listening on the new address

    if (!xhci_slot_fix_max_packet0(slot) || !usb_device_configure(dev)) {
        xhci_slot_free(slot);
        return NULL;
    }
    return slot;
}

bool xhci_device_attach(xhci_hc_t *hc, uint8_t port, uint8_t speed) {
    xhci_slot_t *slot = xhci_device_enumerate(hc, port, speed, NULL, 0);
    if (!slot)
        return false;
    hc->port_slot[port] = slot->id;
    return true;
}

static uint8_t xhci_slot_speed(usb_speed_t speed) {
    switch (speed) {
        case USB_SPEED_LOW:  return XHCI_SPEED_LOW;
        case USB_SPEED_FULL: return XHCI_SPEED_FULL;
        case USB_SPEED_HIGH: return XHCI_SPEED_HIGH;
        default:             return XHCI_SPEED_SUPER;
    }
}

usb_device_t *xhci_device_add(usb_device_t *hub, uint8_t port, usb_speed_t speed) {
    xhci_slot_t *parent = hub->hc_priv;
    xhci_slot_t *slot   = xhci_device_enumerate(parent->hc, parent->port, xhci_slot_speed(speed),
                                                hub, port);
    return slot ? &slot->dev : NULL;
}

// the controller schedules around a hub so it has to be told it is one (4.5.1)
int xhci_hub_setup(usb_device_t *dev, uint8_t ports, uint8_t think_time) {
    xhci_slot_t *slot = dev->hc_priv;

    uint32_t *sctx = xhci_in_ctx(slot, XHCI_IN_SLOT);
    sctx[0] |= XHCI_SLOT_HUB;
    sctx[1] |= XHCI_SLOT_PORTS(ports);
    if (slot->speed == XHCI_SPEED_HIGH)
        sctx[2] |= XHCI_SLOT_TT_THINK(think_time);

    uint32_t *ctrl = xhci_in_ctx(slot, XHCI_IN_CTRL);
    ctrl[0] = 0;
    ctrl[1] = XHCI_ADD_SLOT;

    return xhci_cmd_configure_ep(slot->hc, slot) == XHCI_CC_SUCCESS ? 0 : USB_ERR;
}

// periodic endpoints get their interval as 2^n * 125us (6.2.3.6)
static uint8_t xhci_ep_interval(uint8_t speed, uint8_t type, uint8_t interval) {
    if (type != XHCI_EP_TYPE_INT_IN && type != XHCI_EP_TYPE_INT_OUT)
        return 0;

    // high speed and up already speak in 2^(n-1) microframes
    if (speed == XHCI_SPEED_HIGH || speed >= XHCI_SPEED_SUPER) {
        if (interval < 1)  interval = 1;
        if (interval > 16) interval = 16;
        return interval - 1;
    }

    // full and low speed give milliseconds, round down to a power of two microframes
    uint32_t frames = (uint32_t)(interval ? interval : 1) * 8;
    uint8_t  exp    = 0;
    while ((2u << exp) <= frames)
        exp++;
    if (exp < 3)  exp = 3;
    if (exp > 10) exp = 10;
    return exp;
}

int xhci_endpoint_add(xhci_slot_t *slot, const usb_endpoint_desc_t *desc, uint8_t burst) {
    xhci_hc_t *hc  = slot->hc;
    uint8_t    dci = xhci_dci(desc->address);
    xhci_ep_t *ep  = &slot->eps[dci];
    bool       in  = desc->address & USB_ENDPOINT_IN;
    uint8_t    xfer = desc->attributes & USB_ENDPOINT_TYPE_MASK;

    uint8_t type;
    if (xfer == USB_ENDPOINT_BULK)
        type = in ? XHCI_EP_TYPE_BULK_IN : XHCI_EP_TYPE_BULK_OUT;
    else if (xfer == USB_ENDPOINT_INTERRUPT)
        type = in ? XHCI_EP_TYPE_INT_IN : XHCI_EP_TYPE_INT_OUT;
    else {
        serial_printf(LOG_WARN "xhci%d: endpoint 0x%02x is a type we dont do\n", hc->index, desc->address);
        return USB_ERR;
    }

    if (ep->used) {
        serial_printf(LOG_WARN "xhci%d: slot %u endpoint 0x%02x is already open\n",
                      hc->index, slot->id, desc->address);
        return USB_ERR;
    }
    if (!xhci_ring_init(hc, &ep->ring)) {
        serial_printf(LOG_ERROR "xhci%d: OOM allocating a transfer ring\n", hc->index);
        return USB_ERR;
    }

    uint16_t mps      = desc->max_packet & USB_ENDPOINT_MPS_MASK;
    uint8_t  interval = xhci_ep_interval(slot->speed, type, desc->interval);
    uint32_t esit     = (type == XHCI_EP_TYPE_INT_IN || type == XHCI_EP_TYPE_INT_OUT)
                      ? (uint32_t)mps * (burst + 1) : 0;
    // the spec suggests 3K for bulk and the max payload for interrupt (4.14.1.1)
    uint16_t avg_trb  = esit ? (uint16_t)esit : 3072;

    uint32_t *ctx = xhci_in_ctx(slot, XHCI_IN_EP(dci));
    memset(ctx, 0, hc->ctx_size);
    ctx[0] = XHCI_EP_INTERVAL(interval) | XHCI_EP_ESIT_HI(esit);
    ctx[1] = XHCI_EP_CERR(3) | XHCI_EP_TYPE(type) | XHCI_EP_BURST(burst) | XHCI_EP_MPS(mps);
    ctx[2] = (uint32_t)(ep->ring.phys | ep->ring.cycle);
    ctx[3] = (uint32_t)((uint64_t)ep->ring.phys >> 32);
    ctx[4] = XHCI_EP_AVG_TRB(avg_trb) | XHCI_EP_ESIT_LO(esit);

    // the slot context says how far into the endpoint list to look
    if (dci > slot->context_entries)
        slot->context_entries = dci;
    uint32_t *sctx = xhci_in_ctx(slot, XHCI_IN_SLOT);
    sctx[0] = (sctx[0] & ~XHCI_SLOT_ENTRIES_MASK) | XHCI_SLOT_ENTRIES(slot->context_entries);

    uint32_t *ctrl = xhci_in_ctx(slot, XHCI_IN_CTRL);
    ctrl[0] = 0;
    ctrl[1] = XHCI_ADD_SLOT | XHCI_ADD_EP(dci);

    serial_printf(LOG_INFO "xhci%d: slot %u adding endpoint 0x%02x as dci %u - type %u, mps %u, "
                  "burst %u, interval %u\n", hc->index, slot->id, desc->address, dci, type,
                  mps, burst, interval);

    if (xhci_cmd_configure_ep(hc, slot) != XHCI_CC_SUCCESS) {
        xhci_ring_free(&ep->ring);
        return USB_ERR;
    }

    ep->used       = true;
    ep->halted     = false;
    ep->dci        = dci;
    ep->address    = desc->address;
    ep->type       = type;
    ep->max_packet = mps;
    return 0;
}
