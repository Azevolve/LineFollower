#include "Dashboard.h"

esp_ctrl_data::esp_ctrl_data(lbv_ctrl_data DATA):
    manual_control  (DATA.manual_control),
    turn_on         (DATA.turn_on),
    l_sp            (int16_t(DATA.l_sp)         /10.0),
    r_sp            (int16_t(DATA.r_sp)         /10.0),
    tau_error       (uint16_t(DATA.tau_error)   /100.0),
    kp              (uint16_t(DATA.kp)          /100.0),
    ki              (uint16_t(DATA.ki)          /100.0),
    kd              (uint16_t(DATA.kd)          /100.0),
    tau_d           (uint16_t(DATA.tau_d)       /100.0),
    speed_ref       (uint16_t(DATA.speed_ref)   /10.0),
    A_              (DATA.A_),
    B_              (DATA.B_),
    C_              (DATA.C_),
    D_              (DATA.D_),
    E_              (DATA.E_),
    F_              (DATA.F_) {}

lbv_status_data::lbv_status_data(esp_status_data DATA):
    l_sp                (int16_t(DATA.l_sp              *10.0)),
    l_speed             (int16_t(DATA.l_speed           *10.0)),
    r_sp                (int16_t(DATA.r_sp              *10.0)),
    r_speed             (int16_t(DATA.r_speed           *10.0)),
    error               (int16_t(DATA.error             *100.0)),
    filtered_error      (int16_t(DATA.filtered_error    *100.0)),
    P                   (int16_t(DATA.P                 *10.0)),
    I                   (int16_t(DATA.I                 *10.0)),
    D                   (int16_t(DATA.D                 *10.0)),
    u                   (int16_t(DATA.u                 *10.0)),
    sat_flag            (   byte(DATA.sat_flag)),
    IR                  (   byte(DATA.IR)),

    A_                  (int16_t(DATA.A_)),
    B_                  (int16_t(DATA.B_)),
    C_                  (int16_t(DATA.C_)),
    D_                  (int16_t(DATA.D_))         {}

esp_ctrl_data Dashboard::get(RAW_DATA DATA){
    lbv_ctrl_data lbv_data;
    memcpy(&lbv_data, &DATA.infos, 30);
    return esp_ctrl_data(lbv_data);
}

RAW_DATA Dashboard::set(esp_status_data DATA){
    lbv_status_data data = DATA;
    RAW_DATA out;
    memcpy(&out.infos, &data, 30);
    return out;
}