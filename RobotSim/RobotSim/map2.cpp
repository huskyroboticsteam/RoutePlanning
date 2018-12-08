//
//  map2.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 12/4/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "obstacle.cpp"
#include "agent2.cpp"

class Map2 : public sf::Drawable, public sf::Transformable {
public:
    Map2 (float w, float h, unsigned int s) {
        width = w;
        height = h;
        scale = s;
        
        showGrid = false;
        
        border.setPrimitiveType(sf::LinesStrip);
        border.resize(5);
        
        border[0] = sf::Vector2f(scale, scale);
        border[1] = sf::Vector2f(scale, scale * width);
        border[2] = sf::Vector2f(scale * height, scale * width);
        border[3] = sf::Vector2f(scale * height, scale);
        border[4] = sf::Vector2f(scale, scale);
        
        border[0].color = BORDER_COLOR;
        border[1].color = BORDER_COLOR;
        border[2].color = BORDER_COLOR;
        border[3].color = BORDER_COLOR;
        border[4].color = BORDER_COLOR;
        
        gridlines.setPrimitiveType(sf::Lines);
        for (int x = 1; x < width; x++) {
            for (int y = 1; y < height; y++) {
                gridlines.append(sf::Vector2f(scale * x, scale * y));
                gridlines.append(sf::Vector2f(scale * x, scale * (y + 1)));
                gridlines.append(sf::Vector2f(scale * x, scale * y));
                gridlines.append(sf::Vector2f(scale * (x + 1), scale * y));
                gridlines[gridlines.getVertexCount() - 4].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 3].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 2].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 1].color = GRID_COLOR;
            }
        }
        
        obstacles.setPrimitiveType(sf::Lines);
    }
    
    void toggleGrid() {
        showGrid = !showGrid;
    }
    
    void readObstaclesFromFile(std::string fileName) {
        std::ifstream file;
        file.open(fileName);
        if (file) {
            std::string line;
            
            while (getline(file, line)) {
                std::istringstream in(line);
                
                float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;
                
                in >> x1 >> y1 >> x2 >> y2;
                
                placeObstacle(x1, y1, x2, y2);
            }
        }
        file.close();
    }
    
    void placeObstacle(float x1, float y1, float x2, float y2) {
        sf::Vertex p1(sf::Vector2f(x1 * scale, y1 * scale));
        p1.color = OBSTACLE_COLOR;
        obstacles.append(p1);
        
        sf::Vertex p2(sf::Vector2f(x2 * scale, y2 * scale));
        p2.color = OBSTACLE_COLOR;
        obstacles.append(p2);
    }
    
    sf::Vertex getValidMove(Agent2 agent, float dx, float dy) {
        
    }
    
    const sf::Color BORDER_COLOR = sf::Color::Black;
    const sf::Color GRID_COLOR = sf::Color(128, 128, 128);
    const sf::Color OBSTACLE_COLOR = sf::Color::Red;
    
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        if (showGrid)
            target.draw(gridlines, states);
        target.draw(border, states);
        target.draw(obstacles, states);
    }
    
    sf::VertexArray border;
    sf::VertexArray gridlines;
    
    bool showGrid;
    
    float width; // in meters
    float height; // in meters
    unsigned int scale; // pixels per meter
    
    sf::VertexArray obstacles;
    
    std::vector<Obstacle> obstacleList();
};
