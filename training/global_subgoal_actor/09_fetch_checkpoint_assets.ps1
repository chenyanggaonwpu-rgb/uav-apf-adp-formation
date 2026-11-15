param([string]$WorkRoot='F:/UAV_Schema6')
. "$PSScriptRoot/Common.ps1"
$root=Set-UavEnvironment $WorkRoot;$python=Get-UavPython $root
& $python -B (Join-Path $PSScriptRoot 'tools/fetch_checkpoints.py') --work-root $root
if($LASTEXITCODE -ne 0){throw 'Checkpoint asset download or verification failed.'}
