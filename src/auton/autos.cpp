#include "config.hpp"
#include "auton/autos.hpp"
#include "subsystem/drive.hpp"

using namespace okapi::literals;
using namespace subsystem;

//blueclose
void blueClosePaths() {
    
}

// auton for blue side closer to double zone
void blueCloseAuto() {
}

//bluefar
void blueFarPaths() {

}

void blueFarAuto() {

}

//redclose
void redClosePaths() {
  drive::moveDistanceProfile(1.5);
}

void redCloseAuto() {

}

//redfar
void redFarPaths() {

}

void redFarAuto() {

}

void redFarAutoParkOnly() {

}





// REMEMBER TO REVERSE
void skills() {
}