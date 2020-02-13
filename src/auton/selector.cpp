#include "auton/selector.hpp"
#include "auton/autons.hpp"
#include "config.hpp"
#include "okapi/api.hpp"

namespace auton {
std::vector<std::string> autonNames({"None", "Red Protected 8",
                                     "Red Unprotected 9", "Blue Protected 8",
                                     "Blue Unprotected 9", "Skills 91pt"});

int selectedAuton = 0;
lv_obj_t *curr_sel_label;

// init the lcd
void init() {
  selectedAuton = 1; // make default red protected

  curr_sel_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(curr_sel_label, autonNames[selectedAuton].c_str());
  lv_obj_align(curr_sel_label, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 20);
}

void displayAuton(std::string name) {
  lv_label_set_text(curr_sel_label, name.c_str());
}

void pollSensor() {
  while (true) {
    if (robot::selectionBtn.get_new_press()) {
      if (selectedAuton + 1 >= autonNames.size()) {
        selectedAuton = 0;
      } else {
        selectedAuton++;
      }
    }

    displayAuton(autonNames[selectedAuton].c_str());
    pros::delay(10);
  }
}

void runAuton() {
  std::string autonName = autonNames[selectedAuton];
  displayAuton(autonName);
  switch (selectedAuton) {
    case 0:
      // do nothing
      break;
    case 1:
      redProtectedEight();
      break;
    case 2:
      redUnprotectedNine();
      break;
    case 3:
      blueProtectedEight();
      break;
    case 4:
      blueUnprotectedNine();
      break;
    case 5:
      progSkills91();
      break;
  }
}


}  // namespace auton