param([string]$WorkRoot='F:/UAV_Schema6',[string]$Run='F_s20261110')
. "$PSScriptRoot/Common.ps1"
$root=Set-UavEnvironment $WorkRoot
if(!(Test-Path -LiteralPath (Join-Path $root 'FINAL_MODEL_PANEL.json'))){throw 'Lock the complete model panel with tools/lock_panel.py before TEST512.'}
Invoke-Uav $WorkRoot @('test','--run',$Run)
Invoke-Uav $WorkRoot @('freeze','--run',$Run)
