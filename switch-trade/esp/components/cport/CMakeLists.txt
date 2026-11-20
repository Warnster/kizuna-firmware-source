# cport: the portable FRLG<->LDN trade brain (rfu/gbaframe/block/reliable/pia_zstd/crypto/ccmp/
# ldn_advert/ldn_auth/ni/wap/wonder_card/mg_link/mg_server/mg_host/rfu_leader/gift_pack/ldn_host/reliable_host/pia_host/charmap/pia_connect/sim), shared with the host build and the
# S3 firmware. Sources are VENDORED into this component (previously referenced by an absolute path into
# an older source tree, which broke any clean checkout). AES/SHA/GCM/CCM via the PSA backend
# (native on mbedTLS 4.x / IDF v6.1).
#
# Key material: no cport source includes ldn_keys.h (only the ldn_keys TYPE, from ldn_advert.h), so the
# component builds without it and keys never enter an image through cport. The station glue
# (components/ldn_join, switch-trade) still compiles the git-ignored header; the distributor takes keys
# at runtime from the app (RAM only).
idf_component_register(
    SRCS
        rfu.c gbaframe.c block.c reliable.c pia_zstd.c crypto.c ccmp.c ldn_advert.c
        ni.c wap.c wonder_card.c mg_link.c mg_server.c mg_host.c rfu_leader.c gift_pack.c ldn_host.c reliable_host.c pia_host.c linkplayer.c trade_host.c proxy_activity.c join_proxy.c charmap.c pia_connect.c sim.c ldn_auth.c aes_backend_psa.c
    INCLUDE_DIRS "."
    REQUIRES mbedtls zstd)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
