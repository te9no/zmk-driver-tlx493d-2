# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

これはZMK (Zephyr Mechanical Keyboard) ファームウェア用のInfineon TLX493D-A2BW 3次元磁気センサードライバーモジュールです。このドライバーはTLX493D-A2BW磁気センサーからの入力を読み取り、Z軸状態依存のマウス操作（通常状態時は中ボタン+移動、押し込み状態時はSHIFT+中ボタン+移動）を生成します。

## アーキテクチャ

### 主要コンポーネント

1. **メインドライバー**: `drivers/input/input_tlx493d.c`
   - I2C通信を使用してTLX493D-A2BW センサーからデータを読み取り
   - A2BW専用初期化シーケンス（改良されたリセット手順）
   - Z軸閾値検出による2状態判定（通常/押し込み）
   - 状態依存behavior binding システム
   - エラー回復とバスリカバリー機能
   - 自動キャリブレーション機能（30秒無動作時）
   - ハイステリシス機能による動き検出
   - A2BW診断レジスタによるデータ整合性チェック
   - パリティビット計算と検証機能
   - フレームカウンター(FRM)による ADC hang-up 検出と復帰

2. **状態管理システム**: `src/tlx493d_state.c`
   - Z軸状態の共有管理（スレッドセーフ）
   - ドライバーとbehavior間の状態同期

3. **カスタムBehavior**: `src/behaviors/behavior_z_axis_morph.c`
   - Z軸状態に応じたbinding切り替え
   - デバイスツリー設定可能なnormal/pressed binding

### デバイスツリー設定

- **センサーバインディング**: `dts/bindings/input/infineon,tlx493d-a2bw.yaml`
- **Behaviorバインディング**: `dts/bindings/behaviors/zmk,behavior-z-axis-morph.yaml`
- **設定項目**:
  - `polling-interval-ms`: センサー読み取り間隔（デフォルト: 10ms）
  - `z-press-threshold`: Z軸押し込み検出閾値（デフォルト: 50）
  - `z-hysteresis`: Z軸ヒステリシス値（デフォルト: 10）
  - `normal-binding`: 通常状態時のbehavior binding
  - `pressed-binding`: 押し込み状態時のbehavior binding

### Kconfig設定

主要な設定オプション：
- `INPUT_TLX493D`: ドライバーの有効化
- `INPUT_TLX493D_POLLING_INTERVAL_MS`: ポーリング間隔
- `INPUT_TLX493D_Z_THRESHOLD`: Z軸スクロール閾値

## 開発コマンド

### ビルド
```bash
# ZMKプロジェクトでモジュールとして使用
west build
```

### デバッグ
ドライバーはZephyrログシステムを使用：
- ログレベル: `CONFIG_INPUT_LOG_LEVEL`
- ログモジュール: `tlx493d`

## 技術的詳細
- /sample/A2BW.txt: TLX493D-A2BWセンサーの詳細な仕様が記載されています。
- /sample/XENSIV_3D_Magnetic_Sensor_TLx493D/: センサーのArduino用のライブラリ・サンプルコードです。

### A2BW初期化シーケンス

1. A2BW専用リセットシーケンス（0xFF×2 → 0x00×2 + 30μs遅延）
2. バージョンレジスタ読み取りと確認
3. 診断レジスタチェック（パリティフラグ確認）
4. CONFIG/MOD1/MOD2レジスタ設定
5. 設定後診断確認
6. データ読み取りテスト
7. キャリブレーション実行

### エラー処理
- I2Cバスリカバリー機能
- 連続エラー時の自動再初期化
- 指数バックオフによるリトライ制御
- 最大5回のバスリカバリー試行

### 入力処理
- デッドゾーン機能（X/Y: 3, Z: 5）
- ハイステリシス閾値: 20
- 相対入力として報告（INPUT_REL_X, INPUT_REL_Y, INPUT_REL_WHEEL）

### 自動キャリブレーション
- 30秒間動きが検出されない場合に自動実行
- 100サンプルの平均値を基準点として設定

## ファイル構造

```
├── CMakeLists.txt          # ビルド設定
├── Kconfig                 # 設定オプションのエントリポイント
├── drivers/
│   ├── input/
│   │   ├── input_tlx493d.c    # メインドライバー実装
│   │   └── input_tlx493d.h    # ドライバーヘッダー
├── dts/bindings/input/
│   └── infineon,tlx493d.yaml  # デバイスツリーバインディング
└── zephyr/
    └── module.yml          # Zephyrモジュール定義
```

## 主要定数とマクロ

### A2BWレジスタアドレス
- `TLV493D_REG_BX_MSB`: 0x00 (X軸データMSB)
- `TLV493D_REG_DIAG`: 0x06 (診断レジスタ)
- `TLV493D_REG_CONFIG`: 0x10 (設定レジスタ)
- `TLV493D_REG_MOD1`: 0x11 (モード1レジスタ)
- `TLV493D_REG_VER`: 0x16 (バージョンレジスタ)

### 設定値
- `Z_PRESS_THRESHOLD_DEFAULT`: 50（Z軸押し込み検出閾値）
- `Z_HYSTERESIS_DEFAULT`: 10（Z軸ヒステリシス）
- `AUTO_RECALIBRATION_TIMEOUT_MS`: 3000ms（自動キャリブレーション）

# ZMK Project Coding Instructions for AI

## Critical Prerequisites

**MANDATORY: Before making ANY implementation changes, you MUST:**

1. **Consult the ZMK Repository Expert**: Query the Deepwiki MCP server (repoName: `zmkfirmware/zmk`) to understand the current implementation and identify potential issues.

2. **Consult the Zephyr Repository Expert**: Since ZMK is Zephyr-based, also query the Deepwiki MCP server (repoName: `zephyrproject-rtos/zephyr`) for Zephyr-specific implementation details when needed.

3. **Share Context**: Provide the MCP server with:
   - Current implementation details
   - Identified problems or requirements
   - Your proposed implementation approach

4. **Validate Implementation Strategy**: Confirm your implementation plan with the ZMK expert before proceeding.

5. **Update Documentation**: After making any changes, always update the README.md with relevant information about new features, usage instructions, or implementation status.

## Why This Process is Essential

- **Pre-trained Knowledge Limitations**: LLM training data may not reflect the latest ZMK architecture, APIs, or best practices
- **Build Failure Prevention**: ZMK has specific conventions and dependencies that may not be obvious from general knowledge
- **Implementation Accuracy**: The ZMK codebase has evolved significantly, and outdated approaches will likely fail

## Remember

**NO ASSUMPTIONS**: Always verify current ZMK practices before implementation. What worked in older versions may not work in current releases.

**EXPERT FIRST**: Consult `zmkfirmware/zmk` and `zephyrproject-rtos/zephyr` repository experts before coding, not after encountering build failures.

## Build Verification

**MANDATORY BUILD CHECK**: After any implementation or modification, verify that the build passes by running the following command:

```bash
west build -p -d build/MKB -b seeeduino_xiao_ble -S "zmk-usb-logging studio-rpc-usb-uart" -- -DSHIELD=MKB_L_RZT -DZMK_CONFIG="/workspaces/zmk-config/zmk-config-MKB-1/" -DZMK_EXTRA_MODULES="/workspaces/zmk-modules/zmk-rgbled-widget/;/workspaces/zmk-modules/zmk-pmw3610-driver;/workspaces/zmk-modules/zmk-dongle-display;/workspaces/zmk-modules/zmk-analog-input-driver;/workspaces/zmk-modules/zmk-driver-hmc5883l;/workspaces/zmk-modules/zmk-input-processor-keytoggle;/workspaces/zmk-modules/zmk-driver-tlx493d-2;/workspaces/zmk-modules/zmk-layout-shift"  -DCONFIG_ZMK_STUDIO=y
```

If build fails, fix the issue and try again until it passes.

Important: Build may take several minutes to complete. Make sure to wait for the build to finish completely before determining if the build is successful.


## Branch Strategy

- For compatibility for user's west.yml, no breaking changes should be made without updating the branch name.
- Current stable branch is `v1`.
- When making breaking changes, bump the branch name to `v2`.
- For changes with no breaking changes, new branches should be created from `v1` branch with prefix `v1-`.