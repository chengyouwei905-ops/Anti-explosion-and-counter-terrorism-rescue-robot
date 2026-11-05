param([string]$Compiler = "gcc", [string[]]$TestNames = @())
$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $motionSources = @(
        "App/Src/app_chassis_motion.c", "App/Src/app_chassis_closeloop.c",
        "Algorithm/Src/alg_motion_profile.c", "Algorithm/Src/alg_chassis_path.c", "Algorithm/Src/alg_pid.c"
    )
    $resultSources = @(
        "Protocol/Src/proto_vision.c", "Protocol/Src/proto_vision_result.c",
        "Algorithm/Src/alg_vision_align.c", "Algorithm/Src/alg_roboarm_kinematics.c"
    )
    $taskIncludes = @("-Itests/vision_stubs", "-Itests/uart_stubs", "-ICore/Inc", "-Itests/stubs",
                      "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc")
    $cases = @(
        @{ Name="imu"; Includes=@("-Itests/imu_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IAlgorithm/Inc");
           Sources=@("Algorithm/Src/alg_ahrs.c") },
        @{ Name="imu_uart_debug"; Includes=@("-Itests/vision_stubs", "-Itests/uart_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_imu_uart_debug.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") },
        @{ Name="work_port"; Includes=@("-Itests/work_port_stubs", "-Itests/vision_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_task_work_port.c", "App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c") + $resultSources },
        @{ Name="bomb_arm_split"; Includes=@("-Itests/work_port_stubs", "-Itests/vision_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_task_work_port.c", "App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c") + $resultSources },
        @{ Name="roboarm"; Includes=@("-Itests/vision_stubs", "-IApp/Inc", "-IBsp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c", "Algorithm/Src/alg_roboarm_kinematics.c") },
        @{ Name="chassis_motion"; Includes=@("-Itests/stubs", "-IApp/Inc", "-IAlgorithm/Inc"); Sources=$motionSources },
        @{ Name="chassis_path"; Includes=@("-Itests/stubs", "-IApp/Inc", "-IAlgorithm/Inc"); Sources=$motionSources },
        @{ Name="uart_it"; Includes=@("-Itests/uart_stubs", "-IBsp/Inc", "-IApp/Inc", "-ICommon/Inc", "-IProtocol/Inc", "-IAlgorithm/Inc");
           Sources=@("Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c", "App/Src/app_chassis.c", "Protocol/Src/proto_chassis.c", "Algorithm/Src/alg_chassis_kinematics.c") },
        @{ Name="task_path"; Includes=$taskIncludes; Sources=@("Protocol/Src/proto_vision.c") + $motionSources },
        @{ Name="task_move12"; Includes=$taskIncludes; Sources=@("Protocol/Src/proto_vision.c") + $motionSources },
        @{ Name="vision_result"; Includes=@("-IProtocol/Inc", "-IAlgorithm/Inc", "-ICommon/Inc"); Sources=$resultSources },
        @{ Name="task_vision"; Includes=$taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources },
        @{ Name="task_distance"; Includes=$taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources },
        @{ Name="task_stop_scan"; Includes=$taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources },
        @{ Name="task_config"; Includes=@("-Itests/work_port_stubs") + $taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "App/Src/app_roboarm.c", "Protocol/Src/proto_roboarm.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources },
        @{ Name="task_camera_test"; Includes=$taskIncludes;
           Sources=@("App/Src/app_vision.c", "App/Src/app_task_work.c", "Bsp/Src/bsp_uart.c", "Common/Src/ring_buffer.c") + $resultSources + $motionSources }
    )
    foreach ($requested in $TestNames) {
        if ($requested -notin $cases.Name) { throw "Unknown test case: $requested" }
    }
    foreach ($case in $cases) {
        $name = $case.Name
        if ($TestNames.Count -gt 0 -and $name -notin $TestNames) { continue }
        $executable = "build/test_$name.exe"
        $compileArgs = @("-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic") +
                       $case.Includes + @("tests/test_$name.c") + $case.Sources + @("-o", $executable, "-lm")
        # Normal protocol regression stays enabled even while firmware defaults to camera test.
        if ($name -eq "task_camera_test") { $compileArgs += "-DTASK_WORK_CAMERA_TEST=1" }
        else { $compileArgs += "-DTASK_WORK_CAMERA_TEST=0" }
        & $Compiler @compileArgs
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
        & (Join-Path $repoRoot $executable)
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
    }
} finally {
    Pop-Location
}
