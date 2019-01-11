//
//  obstacle.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 12/4/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//
#ifndef ROBOTSIM_OBSTACLE_H
#define ROBOTSIM_OBSTACLE_H

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>

class Obstacle : public sf::Drawable {
public:
    Obstacle(float p1x, float p1y, float p2x, float p2y, float mapScale, bool dispQ = false);
    
    void recolor(sf::Color newColor);
    
    float x1;
    float y1;
    float x2;
    float y2;
    
    std::vector<std::pair<int, int>> quadrants;
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    
    bool drawQuadrants;
    sf::VertexArray quadrantFill;
    sf::VertexArray line;
    
    float scale;
};

#endif