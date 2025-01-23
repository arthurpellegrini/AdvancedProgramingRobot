#include "VisualizeDataHandler.h"

void VisualizeDataHandler() {
            // Call the Python script to visualize the data
    while (true) {

        system("python3 ./scripts/save_visualization.py");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        } // Update visualization every second
}