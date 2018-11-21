//
//  map.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 11/13/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>

class Map : public sf::Drawable, public sf::Transformable {
    
public:

    Map(float width, float height, int scale) {
        widthInMeters = width;
        heightInMeters = height;
        pixelsPerMeter = scale;
        
        gridOutline.setPrimitiveType(sf::LineStrip);
        gridOutline.resize(5);
        
        gridOutline[0] = sf::Vector2f(scale, scale);
        gridOutline[1] = sf::Vector2f(scale, scale * width);
        gridOutline[2] = sf::Vector2f(scale * height, scale * width);
        gridOutline[3] = sf::Vector2f(scale * height, scale);
        gridOutline[4] = sf::Vector2f(scale, scale);
        
        gridOutline[0].color = OUTLINE_COLOR;
        gridOutline[1].color = OUTLINE_COLOR;
        gridOutline[2].color = OUTLINE_COLOR;
        gridOutline[3].color = OUTLINE_COLOR;
        gridOutline[4].color = OUTLINE_COLOR;
        
        obstacles.setPrimitiveType(sf::Lines);
        
        obstacleCount = 0;
    }
    
    void placeObstacle(float startX, float startY, float endX, float endY) {
        sf::Vertex p1(sf::Vector2f(startX * pixelsPerMeter, startY * pixelsPerMeter));
        p1.color = OBSTACLE_COLOR;
        obstacles.append(p1);
        
        sf::Vertex p2(sf::Vector2f(endX * pixelsPerMeter, endY * pixelsPerMeter));
        p2.color = OBSTACLE_COLOR;
        obstacles.append(p2);
        
        obstacleCount++;
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
    
    sf::Vector2f getOrigin() { return sf::Vector2f(pixelsPerMeter, pixelsPerMeter); }
    int getScale() { return pixelsPerMeter; }
    float getWidth() { return widthInMeters; }
    float getHeight() { return heightInMeters; }
    
    int getObstacleCount() { return obstacleCount; }
    
    const sf::Color OUTLINE_COLOR = sf::Color::Black;
    const sf::Color OBSTACLE_COLOR = sf::Color::Red;
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        target.draw(gridOutline, states);
        target.draw(obstacles, states);
    }
    
    sf::VertexArray gridOutline;
    
    sf::VertexArray obstacles;
    int obstacleCount;
    
    float widthInMeters;
    float heightInMeters;
    int pixelsPerMeter;
};
