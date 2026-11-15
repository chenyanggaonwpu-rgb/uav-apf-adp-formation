param([Parameter(Mandatory=$true)][string]$MatlabExe,[string]$WorkRoot='F:/UAV_Schema6')
. "$PSScriptRoot/Common.ps1"
$root=Set-UavEnvironment $WorkRoot
if(!(Test-Path -LiteralPath $MatlabExe)){throw 'Pass the actual MATLAB executable path.'}
$matlabFolder=(Join-Path $PSScriptRoot 'matlab').Replace("'","''")
& $MatlabExe -batch "addpath('$matlabFolder'); build_worker" -logfile (Join-Path $root 'launcher_logs/build_worker.log')
if($LASTEXITCODE -ne 0){throw 'MATLAB Compiler build failed.'}
Write-Host 'Worker changed. Record the new hash, run all engineering checks and use a fresh run.'
