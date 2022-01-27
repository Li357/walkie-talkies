#include "cy_wcm.h"
#include "util.h"

void connect_to_network(uint8_t ssid[], uint8_t password[]) {
    LOG("Initializing wifi connection manager");
    cy_wcm_config_t config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    cy_wcm_init(&config);

    LOG("Initialized wifi connection manager!");
    cy_wcm_connect_params_t connect_params;
    memset(&connect_params, 0, sizeof(connect_params));
    memcpy(connect_params.ap_credentials.SSID, ssid, sizeof(ssid));

    if (password[0]) {
        memcpy(connect_params.ap_credentials.password, password, sizeof(password));
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA2_AES_PSK;
    } else {
        // no password, assume no security
        connect_params.ap_credentials.security = CY_WCM_SECURITY_OPEN;
    }

    cy_wcm_ip_address_t ip_address;
    cy_rslt_t result = cy_wcm_connect_ap(&connect_params, &ip_address);
    if (result == CY_RSLT_SUCCESS) {
        LOGF("Successfully connected to wifi network %s", connect_params.ap_credentials.SSID);
    } else {
        LOG("Initialization failed :(");
    }
}