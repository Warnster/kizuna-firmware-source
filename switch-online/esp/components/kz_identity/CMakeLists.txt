# Per-adapter identity key + CA-signed certificate. See include/kz_identity.h.
idf_component_register(SRCS "kz_identity.c" "kz_identity_core.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash mbedtls kz_sign)
