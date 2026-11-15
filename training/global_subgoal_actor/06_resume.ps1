param([string]$WorkRoot='F:/UAV_Schema6',[string]$Run='F_s20261110')
. "$PSScriptRoot/Common.ps1"
Start-Uav $WorkRoot $Run $true ''
