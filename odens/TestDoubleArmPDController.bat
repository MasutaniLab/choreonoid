@echo off
:DoubleArmPDController.cnoid�̃e�X�g

:�l�[�~���O�T�[�r�X�̊m�F
rtls /localhost > nul
if errorlevel 1 (
  echo �l�[�~���O�T�[�o��������܂���
  pause
  exit /b 1
  rem /b�I�v�V�����͐e���I��点�Ȃ����߂ɕK�{
)

:��ƃf�B���N�g�����o�b�`�t�@�C���̂���ꏊ�֕ύX
cd %~dp0

:Choreonoid�N��
start "Choreonoid" choreonoid DoubleArmPDController.cnoid

:TkSlider
start "TkSlider" python TkSlider.py

:�R���|�[�l���g����ϐ���
set c=/localhost/DoubleArmV7-DoubleArmV7PDControllerIoRTC.rtc
set s=/localhost/TkSlider0.rtc

:�R���|�[�l���g�N���҂�
:rtls-c
echo %c%�̋N���҂�
timeout 1 /nobreak > nul
rtls %c% > nul 2>&1
if errorlevel 1 goto rtls-c

:rtls-s
echo %s%�̋N���҂�
timeout 1 /nobreak > nul
rtls %s% > nul 2>&1
if errorlevel 1 goto rtls-s

rtcon %s%:slider %c%:qt

rtact %s% %c%

echo Choreonoid�ŃV�~�����[�V�������J�n���Ă��������D

:loop
set /p ans="�I�����܂����H (y/n)"
if not "%ans%"=="y" goto loop

:�f�B�A�N�e�B�x�[�g
rtdeact %s% %c%

:�I���irtexit�́C����������j
rtexit %s%

:Choreonoid�I��
taskkill /im choreonoid.exe
