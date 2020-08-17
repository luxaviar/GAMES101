#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        Vector2D segment = end - start;
        float len = segment.norm();
        Vector2D dir = segment / len;
        float step = len / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i) {
            Mass* mass = new Mass(start + dir * (step * i), node_mass, false);
            masses.push_back(mass);
            if (i > 0) {
                Spring* spring = new Spring(masses[i - 1], mass, k);
                springs.push_back(spring);
            }
        }

//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        constexpr float kd = 0.00005;
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto segment = (s->m2->position - s->m1->position);
            auto len = segment.norm();
            auto f = s->k * segment / len * (len - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                m->forces += gravity * m->mass;
                m->forces += -kd * m->velocity;

                Vector2D a = m->forces / m->mass;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        constexpr float kd = 0.00005;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto segment = (s->m2->position - s->m1->position);
            auto len = segment.norm();
            auto mov = (len - s->rest_length) * segment / len / 2;
            if (!s->m1->pinned)
                s->m1->position += mov;
            if (!s->m2->pinned)
                s->m2->position -= mov;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1 - kd) * (temp_position - m->last_position) + gravity * delta_t * delta_t;
                m->last_position = temp_position;
            }
        }
    }
}
