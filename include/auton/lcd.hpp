#if !defined(_LCD_HPP_)
#define _LCD_HPP_

namespace lcd {

    void selectAutonLog();

    void initButtons();

    lv_res_t selectBlueClose(lv_obj_t *btn);
    lv_res_t selectBlueFar(lv_obj_t *btn);
    lv_res_t selectRedClose(lv_obj_t *btn);
    lv_res_t selectRedFar(lv_obj_t *btn);

    void runAuton();

    void generatePaths(void* param);

    typedef enum {
        eblueClose = 0,
        eblueFar,
        eredClose,
        eredFar,
        eNone
    } tAutons;
}

#endif // _LCD_HPP_
