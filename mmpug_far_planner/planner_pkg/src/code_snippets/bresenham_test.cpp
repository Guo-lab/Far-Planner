/**
 * Play this in https://cpp.sh/
 */
#include <iostream>
#include <vector>
#include <cmath>

/** 
 * Function to implement Bresenham's line algorithm
 */
std::vector<std::pair<int, int>> Bresenham(int x0, int y0, int x1, int y1) {
    std::vector<std::pair<int, int>> points;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        points.push_back({x, y});

        if (x == x1 && y == y1) break;

        int e2 = 2 * err;

        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }

        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    return points;
}

/**
 * Function to print the grid with the line 
 */ 
void PrintGrid(const std::vector<std::pair<int, int>>& points, int width, int height) {
    std::vector<std::vector<char>> grid(height, std::vector<char>(width, '.'));

    for (const auto& p : points) {
        int x = p.first;
        int y = p.second;
        grid[y][x] = '*';
    }

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            std::cout << grid[i][j] << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    int x0, y0, x1, y1;

    std::cout << "Enter starting point (x0 y0): ";
    std::cin >> x0 >> y0;

    std::cout << "Enter ending point (x1 y1): ";
    std::cin >> x1 >> y1;

    std::vector<std::pair<int, int>> line = Bresenham(x0, y0, x1, y1);

    /** Assuming a grid size of 10x10 for visualization */
    int gridWidth = 10;
    int gridHeight = 10;

    PrintGrid(line, gridWidth, gridHeight);

    return 0;
}
