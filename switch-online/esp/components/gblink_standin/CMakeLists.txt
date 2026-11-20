# gblink_standin: C ports of GB-Link-Switch-LDN's Celio cable translators (web/js/cable/*.js, commit c972db4, AGPL-3.0;
# see NOTICE). Pure logic: no RTOS, no malloc, no printf; host-tested in host_tests/run.sh.
idf_component_register(SRCS "ct_common.c" "ct_link.c" "ct_leader.c" "ct_steps.c" "ct_reverse.c" "ct_translator.c"
                       INCLUDE_DIRS "include")
