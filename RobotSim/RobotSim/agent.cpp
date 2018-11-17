//
//  agent.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 11/13/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "map.cpp"

#define PI 3.141592654

class Agent : public sf::Drawable, public sf::Transformable {
    
public:
    Agent(sf::Vector2f startPos, int scale, sf::Color color = sf::Color::Green) {
        xPos = 0.f;
        yPos = 0.f;
        rotation = 45.f;
        
        shape = sf::CircleShape(SHAPE_RADIUS, 3); // equilateral triangle
        shape.setOrigin(SHAPE_RADIUS, SHAPE_RADIUS);
        shape.setFillColor(color);
        shape.setPosition(startPos);
        shape.setRotation(135.f);
        
        path.setPrimitiveType(sf::Lines);
        
        pathIndex = 0;
        
        pixelsPerMeter = scale;
        
        pathColor = color;
    }
    
    void move(float distance) {
        float xOffset = distance * cos(rotation * PI / 180);
        float yOffset = distance * sin(rotation * PI / 180);
        
        xPos += xOffset;
        yPos += yOffset;
        
        sf::Vertex start = shape.getPosition();
        start.color = pathColor;
        
        path.append(start);
        
        shape.move(xOffset * pixelsPerMeter, yOffset * pixelsPerMeter);
        
        sf::Vertex end = shape.getPosition();
        end.color = pathColor;
        
        path.append(end);
        
        pathIndex++;
        std::cout << "Path index: " + std::to_string(pathIndex) << std::endl;
    }
    
    void rotate(float rotate) {
        rotation += rotate;
        shape.rotate(rotate);
    }
    
    void place(float x, float y, float rot, bool drawPath = true) {
        xPos = x;
        yPos = y;
        rotation = rot;
        
        // update shape
        shape.setRotation(90.f + rot);
        
        if (drawPath) {
            sf::Vertex start = shape.getPosition();
            start.color = pathColor;
            path.append(start);
        }
        shape.setPosition((x + 1.f) * pixelsPerMeter, (y + 1.f) * pixelsPerMeter);
        if (drawPath) {
            sf::Vertex end = shape.getPosition();
            end.color = pathColor;
            path.append(end);
        }
    }
    
    float getX() { return xPos; }
    float getY() { return yPos; }
    float getRotation() { return rotation; }
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        target.draw(path, states);
        target.draw(shape, states);
    }
    
    sf::CircleShape shape;
    sf::VertexArray path;
    
    int pathIndex;
    
    float xPos;
    float yPos;
    float rotation;
    
    int pixelsPerMeter;
    
    const float SHAPE_RADIUS = 20.f;
    //const sf::Color SHAPE_COLOR = sf::Color::Green;
    sf::Color pathColor;
};
