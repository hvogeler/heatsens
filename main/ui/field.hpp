#pragma once

#include "lvgl.h"

/**
 * @brief LVGL Field widget - displays a label-value pair.
 *
 * This class wraps LVGL objects to create a reusable field component
 * with a label and a value display, using RAII for automatic cleanup.
 */
class Field {
private:
    lv_obj_t* container_;    ///< Container object with flex layout
    lv_obj_t* label_;        ///< Label showing field name
    lv_obj_t* value_label_;  ///< Label showing field value

public:
    /**
     * @brief Construct a new Field object.
     * @param parent Parent LVGL object.
     * @param label_text Text for the field label.
     * @param initial_value Initial value to display (default: empty).
     */
    Field(lv_obj_t* parent, const char* label_text, const char* initial_value = "");

    /**
     * @brief Set the position of the field container.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @return Reference to this Field for method chaining.
     */
    Field& setPosition(lv_coord_t x, lv_coord_t y);

    /**
     * @brief Update the field value with a string.
     * @param value New value to display.
     */
    void setValue(const char* value);

    /**
     * @brief Update the field value with an integer.
     * @param value New value to display.
     */
    void setValue(int value);

    /**
     * @brief Update the field value with a boolean.
     * @param value New value to display (shows "true" or "false").
     */
    void setValue(bool value);

    /**
     * @brief Get the underlying container object.
     * @return Pointer to the LVGL container object.
     */
    lv_obj_t* getContainer() const { return container_; }

    /**
     * @brief Get the value label object for direct manipulation.
     * @return Pointer to the value label LVGL object.
     */
    lv_obj_t* getValueLabel() const { return value_label_; }

    /**
     * @brief Destructor - automatically cleans up LVGL objects.
     */
    ~Field();

    // Prevent copying
    Field(const Field&) = delete;
    Field& operator=(const Field&) = delete;

    // Allow moving
    Field(Field&& other) noexcept;
    Field& operator=(Field&& other) noexcept;
};
