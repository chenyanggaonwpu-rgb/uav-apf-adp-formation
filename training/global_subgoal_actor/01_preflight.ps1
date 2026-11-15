param([string]$WorkRoot='F:/UAV_Schema6',[ValidateSet('cuda','cpu')][string]$Device='cuda')
. "$PSScriptRoot/Common.ps1"
$checkArguments=@('selftest','--device',$Device)
if($Device -eq 'cpu'){$checkArguments+='--diagnostic'}
Invoke-Uav $WorkRoot $checkArguments
Invoke-Uav $WorkRoot @('prepare','--device',$Device)
