#include "agent.h"

Agent::Agent(unsigned int mapScale, float startX, float startY, float startR, float tSpeed, float rSpeed)
{
        xPos = startX;
        yPos = startY;
        rotation = startR;
        transSpeed = tSpeed;
        rotSpeed = rSpeed;

        scale = mapScale;
        float fScale = (float)scale;

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

void Agent::move(float dx, float dy)
{
        xPos += dx;
        yPos += dy;

        path.append(shapeBase.getPosition());

        shapeBase.move(dx * scale, dy * scale);
        shapeTop.move(dx * scale, dy * scale);

        path.append(shapeBase.getPosition());

        path[path.getVertexCount() - 2].color = PATH_COLOR;
        path[path.getVertexCount() - 1].color = PATH_COLOR;
}

void Agent::rotate(float dr)
{
        rotation += dr;

        shapeBase.rotate(dr);
        shapeTop.rotate(dr);
}

void Agent::clearPath() { path.clear(); }

void Agent::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
        states.transform *= getTransform();

        target.draw(path, states);
        target.draw(shapeBase, states);
        target.draw(shapeTop, states);
}
