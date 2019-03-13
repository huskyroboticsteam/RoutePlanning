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

#include "utils.hpp"

#define PI 3.141592654

class Agent : public sf::Drawable, public sf::Transformable {
public:
    Agent(unsigned int mapScale, float mapW, float mapH, float startX = 0.f, float startY = 0.f, float startR = 0.f, float tSpeed = 1.f / 6.f, float rSpeed = 20.f / 6.f);
    void move(float dx, float dy);
    void rotate(float dr);
    void clearPath();
    float getX() const { return xPos; }
    float getY() const { return yPos; } 
    float getInternalRotation() const { return rotation; } 
    float getTSpeed() const { return transSpeed; }
    float getRSpeed() const { return rotSpeed; }
    float rotateTowards(float x, float y);
    float drive(float speed = 1.f);
    float driveTowards(float targetX, float targetY);
    float turn(float speed = 1.f);
    float turnTowards(float targetAngle);
    float turnTowards(float targetX, float targetY);
    void scaleSpeed(float ss);

    float bot_width;
    
    std::array<RP::polarPoint, 4> getHitBox() { return hitBox; }
    
#if THEME == 0
    const sf::Color BASE_COLOR = sf::Color(55, 22, 126);
    const sf::Color TOP_COLOR =  sf::Color(233, 213, 163);
    // const sf::Color PATH_COLOR = sf::Color(55, 22, 126);
#else
    const sf::Color BASE_COLOR = sf::Color(0, 255, 0);
    const sf::Color TOP_COLOR = sf::Color(0, 0, 0);
    // const sf::Color PATH_COLOR = sf::Color(32, 193, 0);
#endif
    
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
    std::array<RP::polarPoint, 4> hitBox;
    
    unsigned int scale;
    float mapWidth;
    float mapHeight;
};
#endif
