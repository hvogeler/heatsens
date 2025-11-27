#include "field.hpp"

Field::Field(lv_obj_t* parent, const char* label_text, const char* initial_value)
    : container_(nullptr), label_(nullptr), value_label_(nullptr)
{
    // Create container with flex layout
    container_ = lv_obj_create(parent);
    lv_obj_set_layout(container_, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(container_, LV_FLEX_ALIGN_START,
                         LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(container_, 0, LV_PART_MAIN);
    lv_obj_set_width(container_, lv_pct(100));
    lv_obj_set_height(container_, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(container_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(container_, 0, LV_PART_MAIN);

    // Create label
    label_ = lv_label_create(container_);
    lv_label_set_text(label_, label_text);
    lv_obj_set_width(label_, lv_pct(30));
    lv_obj_set_style_text_font(label_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    lv_obj_set_style_pad_all(label_, 0, LV_PART_MAIN);

    // Create value label
    value_label_ = lv_label_create(container_);
    lv_label_set_text(value_label_, initial_value);
    lv_obj_set_style_text_font(value_label_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_margin_left(value_label_, 10, LV_PART_MAIN);
    lv_obj_set_style_text_color(value_label_,
                                lv_palette_lighten(LV_PALETTE_LIGHT_GREEN, 1),
                                LV_PART_MAIN);
    lv_obj_set_style_pad_all(value_label_, 0, LV_PART_MAIN);
}

Field& Field::setPosition(lv_coord_t x, lv_coord_t y)
{
    lv_obj_set_pos(container_, x, y);
    return *this;
}

void Field::setValue(const char* value)
{
    lv_label_set_text(value_label_, value);
}

void Field::setValue(int value)
{
    lv_label_set_text_fmt(value_label_, "%d", value);
}

void Field::setValue(bool value)
{
    lv_label_set_text(value_label_, value ? "true" : "false");
}

Field::~Field()
{
    if (container_) {
        lv_obj_delete(container_);
    }
}

Field::Field(Field&& other) noexcept
    : container_(other.container_)
    , label_(other.label_)
    , value_label_(other.value_label_)
{
    other.container_ = nullptr;
    other.label_ = nullptr;
    other.value_label_ = nullptr;
}

Field& Field::operator=(Field&& other) noexcept
{
    if (this != &other) {
        // Clean up existing resources
        if (container_) {
            lv_obj_delete(container_);
        }

        // Move resources
        container_ = other.container_;
        label_ = other.label_;
        value_label_ = other.value_label_;

        other.container_ = nullptr;
        other.label_ = nullptr;
        other.value_label_ = nullptr;
    }
    return *this;
}
