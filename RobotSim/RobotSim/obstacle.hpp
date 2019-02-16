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
#include "utils.hpp"

class Obstacle : public sf::Drawable {
public:
    Obstacle(float p1x, float p1y, float p2x, float p2y, float mapScale, float mapW, float mapH);
    
    void recolor(sf::Color newColor);
    
    float x1;
    float y1;
    float x2;
    float y2;
    
    std::vector<std::pair<int, int>> quadrants;
    
#if THEME == 0
    const sf::Color color = sf::Color(0, 0, 0);
#else
    const sf::Color color = sf::Color(0, 255, 0);
#endif
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    
    bool drawQuadrants;
    sf::VertexArray quadrantFill;
    sf::VertexArray line;
    
    float scale;
    float mapWidth;
    float mapHeight;
};

#endif
