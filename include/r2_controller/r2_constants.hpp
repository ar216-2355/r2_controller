// 定数の定義をまとめるヘッダ

#pragma once
#include <cstdint>

namespace Mode {
    constexpr uint8_t CURRENT  = 0; // 電流制御
    constexpr uint8_t VELOCITY = 1; // 速度制御
    constexpr uint8_t POSITION = 2; // 位置制御
    constexpr uint8_t DISABLE  = 3; // 無効（脱力）
}

namespace MotorId {
    // --- オムニ4輪 ---
    constexpr uint8_t OMUNI_LF = 1; // 左前
    constexpr uint8_t OMUNI_LB = 2; // 左後
    constexpr uint8_t OMUNI_RB = 3; // 右後
    constexpr uint8_t OMUNI_RF = 4; // 右前
    
    // --- アーム ---
    constexpr uint8_t ARM_LF = 5; // アーム左前
    constexpr uint8_t ARM_LB = 6; // アーム左後
    constexpr uint8_t ARM_RB = 7; // アーム右後
    constexpr uint8_t ARM_RF = 8; // アーム右前

    // --- アームベルト ---
    constexpr uint8_t BELT_F = 9;  // ベルト前
    constexpr uint8_t BELT_B = 10; // ベルト後

    // --- 昇降 ---
    constexpr uint8_t ELEVATOR = 11;
}

struct MotorData {
    float angle = 0.0;     // 累積回転角度 [deg]
    float velocity = 0.0;  // 回転速度 [rpm]
    int16_t torque = 0;  // トルク電流値 (M3508/M2006の生値)
};