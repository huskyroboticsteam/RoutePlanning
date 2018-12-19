//
//  agent2.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 12/8/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <array>

#define PI 3.141592654

class Agent2 : public sf::Drawable, public sf::Transformable {
public:
    Agent2(unsigned int mapScale, float startX = 0.f, float startY = 0.f, float startR = 0.f, float tSpeed = 1.f, float rSpeed = 1.f) {
        xPos = startX;
        yPos = startY;
        rotation = startR;
        transSpeed = tSpeed;
        rotSpeed = rSpeed;
        
        scale = mapScale;
        float fScale = (float) scale;
        
        shapeBase = sf::RectangleShape(sf::Vector2f(scale, 1.5 * scale));
        shapeBase.setOrigin(scale * 0.5f, scale * 0.75f);
        shapeBase.setFillColor(BASE_COLOR);
        shapeBase.setPosition((xPos + 1) * scale, (yPos + 1) * scale);
        shapeBase.setRotation(90 + startR);
        
        shapeTop = sf::CircleShape(scale * 0.5f, 3);
        shapeTop.setOrigin(scale * 0.5f, scale * 0.5f);
        shapeTop.setFillColor(TOP_COLOR);
        shapeTop.setPosition((xPos + 1) * scale, (yPos + 1) * scale);
        shapeTop.setRotation(90 + startR);
        
//        hitBox.push_back({-15.f / fScale, -20.f / fScale, -15.f / fScale, 20.f / fScale});
//        hitBox.push_back({-15.f / fScale, 20.f / fScale, 15.f / fScale, 20.f / fScale});
//        hitBox.push_back({15.f / fScale, 20.f / fScale, 15.f / fScale, -20.f / fScale});
//        hitBox.push_back({15.f / fScale, -20.f / fScale, -15.f / fScale, -20.f / fScale});

        hitBox[0] = {.9f, atan2(.75f, -.5f)};
        hitBox[1] = {.9f, atan2(.75f, .5f)};
        hitBox[2] = {.9f, atan2(-.75f, .5f)};
        hitBox[3] = {.9f, atan2(-.75f, -.5f)};
        
        path.setPrimitiveType(sf::Lines);
    }
    
    void move(float dx, float dy) {
        xPos += dx;
        yPos += dy;
        
        path.append(shapeBase.getPosition());
        
        shapeBase.move(dx * scale, dy * scale);
        shapeTop.move(dx * scale, dy * scale);
        
        path.append(shapeBase.getPosition());
        
        path[path.getVertexCount() - 2].color = PATH_COLOR;
        path[path.getVertexCount() - 1].color = PATH_COLOR;
    }
    
    void rotate(float dr) {
        rotation += dr;
        
        shapeBase.rotate(dr);
        shapeTop.rotate(dr);
    }
    
    void clearPath() { path.clear(); }
    
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
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        target.draw(path, states);
        target.draw(shapeBase, states);
        target.draw(shapeTop, states);
    }
    
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
};
