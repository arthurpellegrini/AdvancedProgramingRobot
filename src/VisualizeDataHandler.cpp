#include "VisualizeDataHandler.h"

/**
 * @file VisualizeDataHandler.cpp
 * @brief Implementation of the function to handle data visualization.
 * 
 * This file contains the implementation of the `VisualizeDataHandler` function, 
 * which calls a Python script periodically to update and save visualizations of data.
 */

/**
 * @brief Continuously visualizes data using an external Python script.
 * 
 * This function runs in a loop and executes a Python script (`save_visualization.py`) 
 * every second to update and save the visualization of the data.
 * 
 * @details The function uses `system` to call the Python script. It pauses 
 * for one second between each call using `std::this_thread::sleep_for`. 
 * This ensures that the visualization is updated at regular intervals.
 * 
 * @warning Ensure that the Python script is in the correct path (`./scripts/save_visualization.py`) 
 * and that the necessary Python environment and dependencies are set up.
 */
void VisualizeDataHandler() {
    // Loop to call the Python script periodically
    while (true) {
        // Execute the Python script
        system("python3 ./scripts/save_visualization.py");

        // Wait for one second before updating the visualization
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
