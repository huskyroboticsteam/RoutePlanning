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
        
        obstacles.setPrimitiveType(sf::LineStrip);
        obstacles.resize(100); // arbitrary max obstacle count: 100
        obstacles[0] = sf::Vector2f(scale, scale);
        obstacles[0].color = OBSTACLE_COLOR;
        
        obstacleCount = 0;
    }
    
    void placeObstacle(sf::Vector2f nextPoint) {
        sf::Vertex* line = &obstacles[obstacleCount + 1];
        
        line[0].position = nextPoint;
        line[0].color = OBSTACLE_COLOR;
        
        obstacleCount++;
    }
    
    sf::Vector2f getOrigin() { return sf::Vector2f(pixelsPerMeter, pixelsPerMeter); }
    int getScale() { return pixelsPerMeter; }
    
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
