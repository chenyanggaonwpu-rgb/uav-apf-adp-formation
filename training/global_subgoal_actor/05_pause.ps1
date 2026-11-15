param([string]$WorkRoot='F:/UAV_Schema6',[string]$Run='F_s20261110')
. "$PSScriptRoot/Common.ps1"
Invoke-Uav $WorkRoot @('pause','--run',$Run)
