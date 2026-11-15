param([string]$WorkRoot='F:/UAV_Schema6',[string]$Run='F_s20261110',[int]$Port=8773)
. "$PSScriptRoot/Common.ps1"
Invoke-Uav $WorkRoot @('monitor','--run',$Run,'--port',"$Port",'--open')
