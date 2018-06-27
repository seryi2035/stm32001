import qbs 1.0

    Product {
    type: "application"
    Depends { name:"cpp" }
    property string mbed: "mbed-src/"
    property string devices: "mbed-devices/"
    property string rtos: "mbed-rtos/"
    property string vendor: "STM"
    property string model: "STM32F1"
    property string cortex: "M3"
    cpp.defines: ["TOOLCHAIN_GCC_CW=1"]
    cpp.positionIndependentCode: false
    cpp.debugInformation: true
    cpp.embedInfoPlist : ""
    cpp.commonCompilerFlags: [
        "-mthumb","-mcpu=cortex-m3","-msoft-float","-mfpu=vfp","-Os",
      "-fdata-sections","-ffunction-sections","-fno-inline","-std=c99","-flto"]
    cpp.linkerFlags:[
        "-flto","-mthumb","-mcpu=cortex-m3","-msoft-float","-mfpu=vfp","--specs=nano.specs","-Wl,--start-group",
        "-Wl,--gc-sections","-T",
        path+"/"+mbed+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/stm32_flash.ld",
        "-lnosys","-lgcc","-lc","-lstdc++", "-lm"]
    cpp.includePaths: [
        mbed+"001/",
        mbed+"CMSIS/CM3/CoreSupport/",
        mbed+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/",
        mbed+"STM32F10x_StdPeriph_Driver/inc/",
        mbed+"STM32F10x_StdPeriph_Driver/src/",
        mbed+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/"
    ]
    files: [
            mbed+"001/*",
            mbed+"CMSIS/CM3/CoreSupport/*",
            mbed+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/*",
            mbed+"STM32F10x_StdPeriph_Driver/inc/*",
            mbed+"STM32F10x_StdPeriph_Driver/src/*",
            mbed+"CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/TrueSTUDIO/*",
            "main.c",
            //"001.c",
            //"001.h"
        ]
    Properties {
        condition: qbs.buildVariant === "debug"
        cpp.defines: outer.concat(["DEBUG=1"])
    }
    Group {
        qbs.install: true
        fileTagsFilter: "application"
    }
}
