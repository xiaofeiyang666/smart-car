# 回退脚本：将关键文件恢复到 20260420_083438 备份版本
Copy-Item "$PSScriptRoot\control.c" "..\..\mdk\control.c" -Force
Copy-Item "$PSScriptRoot\camera.c"  "..\..\code\camera.c" -Force
Copy-Item "$PSScriptRoot\camera.h"  "..\..\code\camera.h" -Force
Write-Output "rollback done"
