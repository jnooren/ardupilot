
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_GETMODE_REQUEST_MAX_SIZE 0
#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_GETMODE_REQUEST_SIGNATURE (0xFD52B9850CCFCACULL)

#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_GETMODE_REQUEST_ID 201





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dsdl_custom_tulip_protocol_heating_GetMode_cxx_iface;
#endif


struct dsdl_custom_tulip_protocol_heating_GetModeRequest {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dsdl_custom_tulip_protocol_heating_GetMode_cxx_iface;
#endif




};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t dsdl_custom_tulip_protocol_heating_GetModeRequest_encode(struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dsdl_custom_tulip_protocol_heating_GetModeRequest_decode(const CanardRxTransfer* transfer, struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dsdl_custom_tulip_protocol_heating_GetModeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg, bool tao);
static inline bool _dsdl_custom_tulip_protocol_heating_GetModeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg, bool tao);
void _dsdl_custom_tulip_protocol_heating_GetModeRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





}

/*
 decode dsdl_custom_tulip_protocol_heating_GetModeRequest, return true on failure, false on success
*/
bool _dsdl_custom_tulip_protocol_heating_GetModeRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_GetModeRequest* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;



    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dsdl_custom_tulip_protocol_heating_GetModeRequest sample_dsdl_custom_tulip_protocol_heating_GetModeRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>



#endif
#endif
