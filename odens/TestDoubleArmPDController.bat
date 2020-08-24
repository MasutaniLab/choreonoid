@echo off
:DoubleArmPDController.cnoidのテスト

:ネーミングサービスの確認
rtls /localhost > nul
if errorlevel 1 (
  echo ネーミングサーバが見つかりません
  pause
  exit /b 1
  rem /bオプションは親を終わらせないために必須
)

:作業ディレクトリをバッチファイルのある場所へ変更
cd %~dp0

:Choreonoid起動
start "Choreonoid" choreonoid DoubleArmPDController.cnoid

:TkSlider
start "TkSlider" python TkSlider.py

:コンポーネント名を変数化
set c=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
set s=/localhost/TkSlider0.rtc

:コンポーネント起動待ち
:rtls-c
echo %c%の起動待ち
timeout 1 /nobreak > nul
rtls %c% > nul 2>&1
if errorlevel 1 goto rtls-c

:rtls-s
echo %s%の起動待ち
timeout 1 /nobreak > nul
rtls %s% > nul 2>&1
if errorlevel 1 goto rtls-s

rtcon %s%:slider %c%:qt

rtact %s% %c%

echo Choreonoidでシミュレーションを開始してください．

:loop
set /p ans="終了しますか？ (y/n)"
if not "%ans%"=="y" goto loop

:ディアクティベート
rtdeact %s% %c%

:終了（rtexitは，引数を一つずつ）
rtexit %s%

:Choreonoid終了
taskkill /im choreonoid.exe
