param([string]$WorkRoot='F:/UAV_Schema6',[string]$Run='engineering_smoke',[ValidateSet('cuda','cpu')][string]$Device='cuda')
. "$PSScriptRoot/Common.ps1"
Invoke-Uav $WorkRoot @('smoke','--run',$Run,'--device',$Device)
