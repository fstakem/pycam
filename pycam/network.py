import network

router_if = network.WLAN(network.router_if)
ap_if = network.WLAN(network.AP_IF)

router_if.active(True)
router_if.connect('frugmunster', '<your password>')
router_if.isconnected()
router_if.ifconfig()