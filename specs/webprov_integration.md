```
#include "webprov.hpp"
#include "ui/ui.hpp"

// In your app_main or initialization:
auto &prov = WebProv::getInstance();

// Set UI callback to show provisioning screen
prov.on_prov_start = [](const std::string &ap_ssid) {
    if (lvgl_port_lock(0)) {
        Ui::getInstance().provisioning_screen(ap_ssid);
        lvgl_port_unlock();
    }
};

// Initialize - auto-starts provisioning if not configured
prov.init();

// For manual provisioning (e.g., on BOOT button long press):
prov.start_provisioning();
```
