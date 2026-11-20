# The user's three LDN values, kept on the adapter across power cycles and updates.
# Write-only from outside: there is no API, op or log that returns them.
idf_component_register(SRCS "kz_keys.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash)
