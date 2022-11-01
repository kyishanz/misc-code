//
// Created by Niu on 7/6/22.
//

#include "subsystemHeaders/library.h"
#include "subsystemHeaders/my-config.h"
#include "subsystemHeaders/amongus.h"
#include "subsystemHeaders/veggies.h"

#define NUMAUTONS 7
LV_IMG_DECLARE(amongus_img_map)
LV_IMG_DECLARE(veggies_img_map)

/**
 * Set motor cartridges.
 */
void initializeMotorCartridge() {
    motor_set_gearing(PORT_DRIVELEFTFRONT, E_MOTOR_GEARSET_06); // blue cartridges
    motor_set_gearing(PORT_DRIVERIGHTFRONT, E_MOTOR_GEARSET_06);
    motor_set_gearing(PORT_DRIVELEFTBACK, E_MOTOR_GEARSET_06);
    motor_set_gearing(PORT_DRIVERIGHTBACK, E_MOTOR_GEARSET_06);
    motor_set_gearing(PORT_DRIVELEFTMIDDLE, E_MOTOR_GEARSET_06);
    motor_set_gearing(PORT_DRIVERIGHTMIDDLE, E_MOTOR_GEARSET_06);
//    motor_set_gearing(PORT_DRIVELEFTFRONT, E_MOTOR_GEARSET_18);
//    motor_set_gearing(PORT_DRIVERIGHTFRONT, E_MOTOR_GEARSET_18);
//    motor_set_gearing(PORT_DRIVELEFTBACK, E_MOTOR_GEARSET_18);
//    motor_set_gearing(PORT_DRIVERIGHTBACK, E_MOTOR_GEARSET_18);
//    motor_set_gearing(PORT_DRIVELEFTMIDDLE, E_MOTOR_GEARSET_18);
//    motor_set_gearing(PORT_DRIVERIGHTMIDDLE, E_MOTOR_GEARSET_18);
    motor_set_gearing(PORT_INTAKE, E_MOTOR_GEARSET_06);
    motor_set_gearing(PORT_FLYWHEEL, E_MOTOR_GEARSET_06);
}

/**
 * Set motor spinning directions (forward or reverse).
 */
void initializeMotorDirection() {
    motor_set_reversed(PORT_DRIVELEFTFRONT, true); // setting false isn't necessary but is added for ease of reading
    motor_set_reversed(PORT_DRIVERIGHTFRONT, false);
    motor_set_reversed(PORT_DRIVELEFTBACK, true);
    motor_set_reversed(PORT_DRIVERIGHTBACK, false);
    motor_set_reversed(PORT_DRIVELEFTMIDDLE, true);
    motor_set_reversed(PORT_DRIVERIGHTMIDDLE, false);
//    motor_set_reversed(PORT_DRIVELEFTFRONT, true);
//    motor_set_reversed(PORT_DRIVELEFTBACK, true);
//    motor_set_reversed(PORT_DRIVELEFTMIDDLE, true);
    motor_set_reversed(PORT_INTAKE, true);
    motor_set_reversed(PORT_FLYWHEEL, true);
}

/**
 * Initializing pneumatics
 */
void initializePneumatics() {
    // can also initialize with 'adi_port_set_config(DIGITAL_SENSOR_PORT, E_ADI_DIGITAL_OUT);'
    pinMode(ANGLETILTER, OUTPUT);
    digitalWrite(ANGLETILTER, LOW);

    pinMode(INDEXER, OUTPUT);
    digitalWrite(INDEXER, LOW);

    pinMode(ENDGAMEF, OUTPUT);
    digitalWrite(ENDGAMEF, LOW);

    pinMode(ENDGAMEB, OUTPUT);
    digitalWrite(ENDGAMEB, LOW);
}

 /**
  * Initializing bumpers
void initializeBumpers() {
     adi_port_set_config(ARMBUMPER, ADI_DIGITAL_IN);
     pinMode(ARMBUMPER, INPUT);

     adi_port_set_config(BACKBUMPER, ADI_DIGITAL_IN);
     pinMode(BACKBUMPER, INPUT);
}
  */

/**
 * Initializing vision
 */
void initializeVision() {
    /** Initializing the vision sensors **/
    // Clears the vision sensor LED color
    vision_clear_led(VISION_FRONT);
    // Sets the (0,0) coordinate for the Field of View --> E_VISION_ZERO_CENTER sets the (0,0) coordinate as the center of the FOV
    vision_set_zero_point(VISION_FRONT, E_VISION_ZERO_CENTER);

    vision_clear_led(VISION_BACK);
    vision_set_zero_point(VISION_BACK, E_VISION_ZERO_CENTER);

    vision_clear_led(VISION);
    vision_set_zero_point(VISION, E_VISION_ZERO_CENTER);

    /** Getting vision signatures **/
    vision_signature_s_t RED_SIG = vision_signature_from_utility(2, 5187, 10159, 7672, -1567, -263, -914, 1.300, 0);
    vision_signature_s_t BLUE_SIG = vision_signature_from_utility(1, -2685, -1207, -1946, 5587, 10159, 7872, 1.900, 0);
    vision_signature_s_t SIG_3 = vision_signature_from_utility(3, 0, 0, 0, 0, 0, 0, 3.000, 0);
    vision_signature_s_t SIG_4 = vision_signature_from_utility(4, 0, 0, 0, 0, 0, 0, 3.000, 0);
    vision_signature_s_t SIG_5 = vision_signature_from_utility(5, 0, 0, 0, 0, 0, 0, 3.000, 0);
    vision_signature_s_t SIG_6 = vision_signature_from_utility(6, 0, 0, 0, 0, 0, 0, 3.000, 0);
    vision_signature_s_t SIG_7 = vision_signature_from_utility(7, 0, 0, 0, 0, 0, 0, 3.000, 0);

    /** Creating vision color codes ASK NIDHYA WHAT THIS DOES + WHEN WE USE THIS **/
    vision_color_code_t COLOR_CODE = vision_create_color_code(VISION, 1, 2, 3, 4, 5);
    vision_color_code_t COLOR_CODE_FRONT = vision_create_color_code(VISION_FRONT, 1, 2, 3, 4, 5);
    vision_color_code_t COLOR_CODE_BACK = vision_create_color_code(VISION_BACK, 1, 2, 3, 4, 5);
}

void initializeGPS() {
    gps_set_offset(GPSFRONT_PORT, FRONT_X_OFFSET, FRONT_Y_OFFSET);
    gps_set_offset(GPSBACK_PORT, BACK_X_OFFSET, BACK_Y_OFFSET);
}

/** Picker **/
static const char *auton_map[] = {"SOLO WP", "HALF WP", "\n",
                                  "SKILLS", "-", "\n",
                                 "-", "-", "\n",
                                 "-", "-", "\n",
                                  "-", "-",
                                                    ""};
static const char *auton_strings[] = {"SOLO WP", "HALF WP", "SKILLS", "-", "-", "-", "-", "-", "-", "-", ""};
static const char *alliance_map[] = {"LEFT", "RIGHT", ""}; // last element has to be "" according to lvgl api
static const char *color_map[] = {"RED", "BLUE", ""};

lv_obj_t* imgbtn;

lv_style_t buttonStyleREL; // released style
lv_style_t buttonStylePR; // pressed style

lv_style_t btnmStyleREL; // released style
lv_style_t btnmStylePR; // pressed style
lv_style_t btnmStyleTogREL; // toggled release style
lv_style_t btnmStyleINA; // toggled release style
lv_style_t btnmStyleBackground; // background color

void printImage() {
    imgbtn = lv_imgbtn_create(lv_scr_act(), NULL);
    lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_REL, &veggies_img_map);
    lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_PR, &amongus_img_map);
    lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_TGL_REL, &amongus_img_map);
    lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_TGL_PR, &veggies_img_map);
    lv_imgbtn_set_toggle(imgbtn, true);
    lv_obj_set_size(imgbtn, 149, 91);
    lv_obj_set_x(imgbtn, 3);
    lv_obj_set_y(imgbtn, 3);
}

void initializeStyle() {
    // regular buttons
    lv_style_copy(&buttonStyleREL, &lv_style_btn_rel);
    lv_color_t background_gray = LV_COLOR_MAKE(51, 51, 51);
    buttonStyleREL.body.main_color = background_gray;
    buttonStyleREL.body.grad_color = background_gray;
    buttonStyleREL.body.border.color = LV_COLOR_MAKE(225, 221, 240);
    buttonStyleREL.text.color = LV_COLOR_MAKE(225, 221, 240);

    lv_style_copy(&buttonStylePR, &lv_style_btn_pr);
    buttonStylePR.body.main_color = LV_COLOR_MAKE(203, 198, 218);
    buttonStylePR.body.grad_color = LV_COLOR_MAKE(203, 198, 218);
    buttonStylePR.body.border.color = LV_COLOR_MAKE(225, 221, 240);
//    buttonStylePR.body.radius = 0;
    buttonStylePR.text.color = LV_COLOR_MAKE(96, 85, 140);

    // button matrix buttons
    lv_style_copy(&btnmStyleREL, &lv_style_btn_rel);
    btnmStyleREL.body.main_color = background_gray;
    btnmStyleREL.body.grad_color = background_gray;
    btnmStyleREL.body.border.width = 0;
    btnmStyleREL.text.color = LV_COLOR_MAKE(225, 221, 240);

    lv_style_copy(&btnmStylePR, &lv_style_btn_pr);
    btnmStylePR.body.main_color = LV_COLOR_MAKE(203, 198, 218);
    btnmStylePR.body.grad_color = LV_COLOR_MAKE(203, 198, 218);
    btnmStylePR.body.border.color = LV_COLOR_MAKE(225, 221, 240);
    btnmStylePR.text.color = LV_COLOR_MAKE(96, 85, 140);

    lv_style_copy(&btnmStyleINA, &lv_style_btn_rel);
    btnmStyleINA.body.main_color = background_gray;
    btnmStyleINA.body.grad_color = background_gray;
    btnmStyleINA.body.border.width = 0;
    btnmStyleINA.text.color = LV_COLOR_MAKE(225, 221, 240);

    lv_style_copy(&btnmStyleTogREL, &lv_style_btn_rel);
    btnmStyleTogREL.body.main_color = background_gray;
    btnmStyleTogREL.body.grad_color = background_gray;
    btnmStyleTogREL.body.border.color = LV_COLOR_MAKE(225, 221, 240);
    btnmStyleTogREL.text.color = LV_COLOR_MAKE(225, 221, 240);

//    lv_style_copy(&btnmStyleBackground, &lv_style_transp);
//    btnmStyleBackground.body.main_color = LV_COLOR_MAKE(54, 54, 54);
//    btnmStyleBackground.body.grad_color = LV_COLOR_MAKE(54, 54, 54);
//    btnmStyleBackground.body.border.color = LV_COLOR_MAKE(225, 221, 240);
//    btnmStyleBackground.text.color = LV_COLOR_MAKE(225, 221, 240);
}

static lv_res_t leftRightClicked(lv_obj_t *btnm, const char *txt) {
    lv_btnm_set_toggle(btnm, true, 1);
    lv_btnm_set_toggle(btnm, true, 2);

    if (strcmp(txt, "LEFT") == 0) { // strcmp compares 2 strings, returns 0 if equal
        autonSide = 0;
    } else if (strcmp(txt, "RIGHT") == 0) {
        autonSide = 1;
    }

    logStringWithFloat("HI", (float) autonSide);

    return LV_RES_OK; /* Return OK because the button matrix is not deleted */
}
lv_obj_t *autonSideMatrix;
void leftRightButtons() {
    // initializing the button matrix
    autonSideMatrix = lv_btnm_create(lv_scr_act(), NULL);

    // buttons in the matrix will be created by string array alliance_map
    lv_btnm_set_map(autonSideMatrix, alliance_map);

    // action to take after button is toggled
    lv_btnm_set_action(autonSideMatrix, leftRightClicked);

    // positioning button matrix
    lv_obj_set_size(autonSideMatrix, LV_HOR_RES / 3 - 10, 40);
    lv_obj_align(autonSideMatrix, imgbtn, LV_ALIGN_OUT_BOTTOM_MID, 6, 0);

    // styling
    lv_btnm_set_style(autonSideMatrix, LV_BTNM_STYLE_BTN_REL, &btnmStyleREL);
    lv_btnm_set_style(autonSideMatrix, LV_BTNM_STYLE_BTN_PR, &btnmStylePR);
    lv_btnm_set_style(autonSideMatrix, LV_BTNM_STYLE_BTN_TGL_REL, &btnmStyleTogREL);
    lv_btnm_set_style(autonSideMatrix, LV_BTNM_STYLE_BTN_INA, &btnmStyleINA);
    lv_btnm_set_style(autonSideMatrix, LV_BTNM_STYLE_BG, &btnmStyleBackground);
}

static lv_res_t colorSelectorClicked(lv_obj_t *btnm, const char *txt) {
    lv_btnm_set_toggle(btnm, true, 1);
    lv_btnm_set_toggle(btnm, true, 2);

    if (strcmp(txt, "RED") == 0) { // strcmp compares 2 strings, returns 0 if equal
        autonColor = 0;
    } else if (strcmp(txt, "BLUE") == 0) {
        autonColor = 1;
    }

    logStringWithFloat("SELECTED", (float) autonColor);

    return LV_RES_OK; /* Return OK because the button matrix is not deleted */
}
lv_obj_t *autonColorMatrix;
void colorButtons() {
    // initializing the button matrix
    autonColorMatrix = lv_btnm_create(lv_scr_act(), NULL);

    // buttons in the matrix will be created by string array alliance_map
    lv_btnm_set_map(autonColorMatrix, color_map);

    // action to take after button is toggled
    lv_btnm_set_action(autonColorMatrix, colorSelectorClicked);

    // positioning button matrix
    lv_obj_set_size(autonColorMatrix, LV_HOR_RES / 3 - 10, 40);
    lv_obj_align(autonColorMatrix, autonSideMatrix, LV_ALIGN_OUT_BOTTOM_MID, 3, 8);

    // styling
    lv_btnm_set_style(autonColorMatrix, LV_BTNM_STYLE_BTN_REL, &btnmStyleREL);
    lv_btnm_set_style(autonColorMatrix, LV_BTNM_STYLE_BTN_PR, &btnmStylePR);
    lv_btnm_set_style(autonColorMatrix, LV_BTNM_STYLE_BTN_TGL_REL, &btnmStyleTogREL);
    lv_btnm_set_style(autonColorMatrix, LV_BTNM_STYLE_BTN_INA, &btnmStyleINA);
    lv_btnm_set_style(autonColorMatrix, LV_BTNM_STYLE_BG, &btnmStyleBackground);
}

static lv_res_t autonSelectorClicked(lv_obj_t *btnm, const char *txt) {
    for (int i = 0; i < sizeof(auton_strings) / sizeof(auton_strings[0]); i++) {
        if (strcmp(auton_strings[i], txt) == 0) {
            autonNumber = i + 1;
            break;
        }
        lv_btnm_set_toggle(btnm, true, autonNumber);
    }

    printf("%s\n", auton_strings[autonNumber - 1]);
    printf("\t%d\n", autonNumber);

    return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

void autonSelectionButtons() {
    lv_obj_t *obj = drawRectangle( LV_HOR_RES / 3 + 5, 2, 2 * LV_HOR_RES / 3 - 5, 233, LV_COLOR_MAKE(225, 221, 240));
    lv_obj_t *autonProgramMatrix = lv_btnm_create(lv_scr_act(), NULL);
    lv_btnm_set_map(autonProgramMatrix, auton_map);
    lv_btnm_set_action(autonProgramMatrix, autonSelectorClicked);
    lv_obj_set_size(autonProgramMatrix,  2 * LV_HOR_RES / 3 - 20, 228);
    lv_obj_align(autonProgramMatrix, imgbtn, LV_ALIGN_OUT_RIGHT_TOP, 20, 2);

    lv_btnm_set_style(autonProgramMatrix, LV_BTNM_STYLE_BTN_REL, &btnmStyleREL);
    lv_btnm_set_style(autonProgramMatrix, LV_BTNM_STYLE_BTN_PR, &btnmStylePR);
    lv_btnm_set_style(autonProgramMatrix, LV_BTNM_STYLE_BTN_TGL_REL, &btnmStyleTogREL);
    lv_btnm_set_style(autonProgramMatrix, LV_BTNM_STYLE_BTN_INA, &btnmStyleINA);
    lv_btnm_set_style(autonProgramMatrix, LV_BTNM_STYLE_BG, &btnmStyleBackground);
}

void printAutonSelected() {
    lv_obj_t* autonSelectedText = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_recolor(autonSelectedText, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_align(autonSelectedText, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/

    char dest_string[] = "#ffffff ";
    strcat(dest_string, auton_strings[autonNumber]);
    strcat(dest_string, " ");
    strcat(dest_string, alliance_map[autonSide]);
    strcat(dest_string, " ");
    strcat(dest_string, color_map[autonColor]);

//    lv_label_set_text(autonSelectedText, dest_string);
    lv_obj_align(autonSelectedText, NULL, LV_ALIGN_CENTER, 0, 0);
}

static lv_res_t lockAutonSelector(lv_obj_t* btn) {
    // if autonSide and autonNumber have been selected
    if (autonSide != 123456 && autonNumber != 123456 && autonColor != 123456) {
        lv_obj_clean(lv_scr_act());
//        printAutonSelected();
    }
    return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

void lockButton() {
    lv_obj_t* lockButton = lv_btn_create(lv_scr_act(), NULL);
    lv_btn_set_action(lockButton, LV_BTN_ACTION_CLICK, lockAutonSelector);
    lv_obj_set_size(lockButton, LV_HOR_RES / 3 - 60, 40);
    lv_obj_t* label = lv_label_create(lockButton, NULL);
    lv_label_set_text(label, "LOCK");
    lv_obj_align(lockButton, autonColorMatrix, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    // style
    lv_btn_set_style(lockButton, LV_BTN_STYLE_REL, &buttonStyleREL);
    lv_btn_set_style(lockButton, LV_BTN_STYLE_PR, &buttonStylePR);
}

void myPicker() {
    delay(100);
    initializeStyle();
    printImage();
    leftRightButtons();
    colorButtons();
    autonSelectionButtons();
    lockButton();
}




//regular initializing stuff

//picker

//grand function that we call
