//
//  agent.h
//  RobotSim
//
//  Gary: used as both controller and view. Holds a Simulator instance as its model.
//
//  Created by Tadeusz Pforte on 11/13/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//
#ifndef ROBOTSIM_AGENT_H
#define ROBOTSIM_AGENT_H

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <array>

#define PI 3.141592654

class Agent : public sf::Drawable, public sf::Transformable {
public:
    Agent(unsigned int mapScale, float mapW, float mapH, float startX = 0.f, float startY = 0.f, float startR = 0.f, float tSpeed = 1.f, float rSpeed = 1.f);
    void move(float dx, float dy);
    void rotate(float dr);
    void clearPath();
    float getX() { return xPos; }
    float getY() { return yPos; }
    float getInternalRotation() { return rotation; }
    float getTSpeed() { return transSpeed; }
    float getRSpeed() { return rotSpeed; }
    
    std::array<std::pair<float, float>, 4> getHitBox() { return hitBox; }
    
    const sf::Color BASE_COLOR = sf::Color(55, 22, 126);
    const sf::Color TOP_COLOR =  sf::Color(233, 213, 163);
    const sf::Color PATH_COLOR = sf::Color(55, 22, 126);
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    sf::RectangleShape shapeBase;
    sf::CircleShape shapeTop;
    sf::VertexArray path;
    
    float xPos;
    float yPos;
    float rotation;
    float transSpeed;
    float rotSpeed;
    
    // stores the four corners of the hitbox as polar coordinates (r, θ)
    // r is in meters, θ is in radians
    std::array<std::pair<float, float>, 4> hitBox;
    
    unsigned int scale;
    float mapWidth;
    float mapHeight;
};
#endif
