# KiCad Schematic 変更手順（4ch → 3ch）

このドキュメントはKiCad GUIで行う具体的な変更手順を示します。

---

## 変更概要

| シート | 変更内容 |
|--------|---------|
| drivers.kicad_sch | CH3削除（A4, J33, C33削除）、STEP/DIR net名更新 |
| encoders_io.kicad_sch | CH3削除（ENC_3ブロック削除） |
| mcu_usb.kicad_sch | GPIO再アサイン、ADC net名変更、ENC3削除 |
| adc_io.kicad_sch | ADCブロック再設計（POT×3 + CURRENT + VIN_MON） |

---

## 1. drivers.kicad_sch

### 削除するコンポーネント
- `A4` (DRV8825_CH3)
- `J33` (MOTOR_CH3)
- `C33` (100µF デカップリングコンデンサ)

### 確認・変更するネット名
現在のnet名（drivers）はSKILL.mdに準拠。以下のnet命名は変更不要：
- STEP_0, STEP_1, STEP_2 → そのまま使用
- DIR_0, DIR_1, DIR_2 → そのまま使用
- MS_0, MS_1, MS_2 → そのまま使用（M0/M1/M2と統一できるが任意）

### 手順
1. drivers.kicad_sch を開く
2. A4 (DRV8825_CH3) を選択して Delete
3. J33 (MOTOR_CH3) を選択して Delete
4. C33 (100µF) を選択して Delete
5. 関連するワイヤーも削除
6. 保存

---

## 2. encoders_io.kicad_sch

### 削除するコンポーネント
- ENC_3 ブロック全体：
  - AM26LV32 の ENC_3 に使用されているユニット
  - ENC_3 コネクタ（JST-XH）
  - ENC_3_EA+/-, ENC_3_EB+/-, ENC_3_EZ+/- の全ワイヤーとラベル

### 手順
1. encoders_io.kicad_sch を開く
2. ENC_3 に関連するすべての要素（差動ペアの+/-、コネクタ、AM26LV32ユニット）を選択して削除
3. 保存

---

## 3. mcu_usb.kicad_sch（主要変更）

### ADC net名の変更

| 変更前 | 変更後 | GPIO |
|--------|--------|------|
| ADC0 | POT0 | GPIO1 |
| ADC1 | POT1 | GPIO2 |
| ADC2 | POT2 | GPIO3 |
| ADC3 | CURRENT | GPIO4（GPIO40から移動） |
| （新規） | VIN_MON | GPIO5 |

### STEP/DIR GPIO変更

| 信号 | 変更前GPIO | 変更後GPIO |
|------|-----------|-----------|
| STEP0 | GPIO4 | GPIO6 |
| DIR0 | GPIO5 | GPIO7 |
| STEP1 | GPIO6 | GPIO8 |
| DIR1 | GPIO7 | GPIO9 |
| STEP2 | GPIO8 | GPIO10 |
| DIR2 | GPIO9 | GPIO11 |
| DRV_EN | GPIO12 | GPIO12（変更なし） |
| DRV_RESET | GPIO13 | GPIO13（変更なし） |
| DRV_SLEEP | GPIO14 | GPIO14（変更なし） |

### エンコーダGPIO変更

| 信号 | 変更前GPIO | 変更後GPIO |
|------|-----------|-----------|
| ENC0_A | GPIO15 | GPIO15（変更なし） |
| ENC0_B | GPIO16 | GPIO16（変更なし） |
| ENC0_Z | 未接続 | GPIO17（新規接続） |
| ENC1_A | GPIO17 | GPIO18 |
| ENC1_B | GPIO18 | GPIO21 |
| ENC1_Z | 未接続 | GPIO35（新規接続） |
| ENC2_A | GPIO21 | GPIO36 |
| ENC2_B | GPIO35 | GPIO37 |
| ENC2_Z | 未接続 | GPIO40（新規接続） |
| ENC3_A | GPIO36 | **削除** |
| ENC3_B | GPIO37 | **削除** |
| ENC3_Z | 未接続 | **削除** |

### 削除するnet接続
- STEP3, DIR3 の接続（GPIO10/GPIO11から切断）
- ENC3_A, ENC3_B, ENC3_Z のラベルすべて
- ADC3 の接続（GPIO40から切断）

### 追加するnet接続
- GPIO4 → CURRENT ラベル
- GPIO5 → VIN_MON ラベル
- GPIO17 → ENC0_Z ラベル
- GPIO35 → ENC1_Z ラベル
- GPIO40 → ENC2_Z ラベル

### 手順
1. mcu_usb.kicad_sch を開く
2. ESP32-S3のGPIO4のラベルを ADC0 から CURRENT に変更
   （または: 古いラベル削除 → 新しいラベル追加）
3. GPIO5: ADC1（もし未接続ならスキップ）→ VIN_MON に変更
4. GPIO6→10に対してSTEP/DIR netラベルを上記テーブルに従い変更
5. ENC0_Z/ENC1_Z/ENC2_Z を各GPIOに接続（GPIO17/35/40）
6. ENC3_A, ENC3_B, ENC3_Z のラベルを削除
7. STEP3, DIR3 のラベルを削除
8. 保存

---

## 4. adc_io.kicad_sch（再設計）

このシートは現在ほぼ空（+3V3/GNDのみ）。以下を追加：

### 追加するブロック

#### ポテンショメーター入力（POT0/1/2）×3
各チャンネル同じ構成：
```
AIN → 20kΩ → POT_n
              10kΩ → GND
POT_n → 0.1µF → GND
POT_n → [net label: POT0/POT1/POT2]
```

#### 電流センス（CURRENT）
```
ISHUNT → op-amp（ゲイン設定） → CURRENT_OUT
CURRENT_OUT → 0.1µF → GND
CURRENT_OUT → [net label: CURRENT]
```

#### 電源電圧モニタ（VIN_MON）

VIN_24V（モーター電源）を分圧して ADC1_CH4（GPIO5）へ入力。

```
VIN_24V → R_top(68kΩ) → VIN_MON_NODE
          R_bot(10kΩ) → GND
VIN_MON_NODE → 0.1µF → GND
VIN_MON_NODE → [net label: VIN_MON]
```

分圧計算：
- 24V × 10k/(68k+10k) = 24 × 0.1282 ≈ **3.08V** → 3.3V ADC フルスケール以内 ✓
- 最大許容電圧（安全マージン）：3.3V × (78k/10k) ≈ 25.7V

ADC 読み取り換算：
```
VIN_24V [V] = ADC_raw / 4095 × 3.3 × (78k/10k)
            = ADC_raw × 0.00628
```

---

## 新GPIO/net対応表（最終確認）

| GPIO | Net名 | シート |
|------|-------|--------|
| GPIO1 | POT0 | adc_io ↔ mcu_usb |
| GPIO2 | POT1 | adc_io ↔ mcu_usb |
| GPIO3 | POT2 | adc_io ↔ mcu_usb |
| GPIO4 | CURRENT | adc_io ↔ mcu_usb |
| GPIO5 | VIN_MON | adc_io ↔ mcu_usb |
| GPIO6 | STEP0 (STEP_0) | drivers ↔ mcu_usb |
| GPIO7 | DIR0 (DIR_0) | drivers ↔ mcu_usb |
| GPIO8 | STEP1 (STEP_1) | drivers ↔ mcu_usb |
| GPIO9 | DIR1 (DIR_1) | drivers ↔ mcu_usb |
| GPIO10 | STEP2 (STEP_2) | drivers ↔ mcu_usb |
| GPIO11 | DIR2 (DIR_2) | drivers ↔ mcu_usb |
| GPIO12 | DRV_EN | drivers ↔ mcu_usb |
| GPIO13 | DRV_RESET | drivers ↔ mcu_usb |
| GPIO14 | DRV_SLEEP | drivers ↔ mcu_usb |
| GPIO15 | ENC0_A | encoders ↔ mcu_usb |
| GPIO16 | ENC0_B | encoders ↔ mcu_usb |
| GPIO17 | ENC0_Z | encoders ↔ mcu_usb |
| GPIO18 | ENC1_A | encoders ↔ mcu_usb |
| GPIO21 | ENC1_B | encoders ↔ mcu_usb |
| GPIO35 | ENC1_Z | encoders ↔ mcu_usb |
| GPIO36 | ENC2_A | encoders ↔ mcu_usb |
| GPIO37 | ENC2_B | encoders ↔ mcu_usb |
| GPIO40 | ENC2_Z | encoders ↔ mcu_usb |
| GPIO41 | M0 (MS_0) | drivers ↔ mcu_usb |
| GPIO42 | M1 (MS_1) | drivers ↔ mcu_usb |
| GPIO45 | M2 (MS_2) | drivers ↔ mcu_usb |

---

## 作業完了後のERC確認項目

1. Run ERC（電気ルールチェック）
2. 未接続ピン（unconnected）がないこと
3. ネット名のタイポがないこと
4. DRV8825_CH0/1/2 の EN/RESET/SLEEP が共通接続されていること
5. USB D+/D- に 22Ω 直列抵抗があること
6. 各ADC入力に RC フィルタがあること
