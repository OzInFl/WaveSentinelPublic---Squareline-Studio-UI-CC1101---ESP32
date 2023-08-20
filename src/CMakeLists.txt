SET(SOURCES screens/ui_scrSplash.c
    screens/ui_scrMain.c
    screens/ui_scrProtAna.c
    screens/ui_scrKybdNum.c
    screens/ui_scrPresets.c
    screens/ui_scrSettings.c
    screens/ui_scrWifiApps.c
    screens/ui_scrWifiScanner.c
    screens/ui_scrRCSWMain.c
    screens/ui_scrRCSWTenPole.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_splashbkgnd2_png.c
    images/ui_img_mainbackground_png.c
    images/ui_img_teslabtnicon_png.c
    images/ui_img_wifilogo_png.c
    images/ui_img_analyzericon_png.c
    images/ui_img_subrecordplay_png.c
    images/ui_img_flippericon_png.c
    images/ui_img_blankpgbkgnd_png.c
    fonts/ui_font_LCD20at8BPP.c
    fonts/ui_font_LCD22at8BPP.c
    fonts/ui_font_LCD24at8BPP.c
    fonts/ui_font_LCD30at8BPP.c
    fonts/ui_font_Verdana11.c
    fonts/ui_font_Verdana12.c
    fonts/ui_font_Verdana14.c
    fonts/ui_font_Verdana16.c
    fonts/ui_font_Verdana18.c
    fonts/ui_font_Verdana24.c
    fonts/ui_font_Verdana8.c)

add_library(ui ${SOURCES})
