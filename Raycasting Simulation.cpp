#include <SFML/Graphics.hpp>
#include <vector>
#include <array>
#include "HSV.h"
const float WIN_WIDTH{ 700 };
const float WIN_HEIGHT{ 700 };
namespace Utility
{

	const float PI = 3.141592654;
	static float Magnitude(const sf::Vector2f& vec)
	{
		return sqrt(pow(vec.x, 2) + pow(vec.y, 2));
	}
	static sf::Vector2f Normalise(const sf::Vector2f& vec)
	{
		auto mag = Utility::Magnitude(vec);
		return sf::Vector2f{ vec.x / mag, vec.y / mag };
	}
	static float CalculateAngle(const sf::Vector2f& vec)
	{
		float rotation = atan(vec.y / vec.x);
		//Y co-ordinate is flipped for canvas!
		if ((vec.y < 0 && vec.x > 0) || (vec.y > 0 && vec.x > 0))
		{
			rotation = PI/2 + rotation;
		}
		else if ((vec.y > 0 && vec.x < 0) || (vec.y < 0 && vec.x < 0)) // tan is negative 3rd quadrant
		{
			rotation = 3*PI/2 + rotation;
		}
		return rotation;
	}

}
//A simple line class with a start and end point.
class Line
{
private:
	sf::VertexArray start_end_points;
	//This property will only be used if the line is in particular, a ray.
	sf::Vector2f goal_point;

public:
	
	Line(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Vector2f& goal, const sf::Color& col)
	{
		start_end_points = sf::VertexArray(sf::PrimitiveType::LineStrip, 2);
		start_end_points[0] = sf::Vertex(start, col);
		start_end_points[1] = sf::Vertex(end, col);
		goal_point = goal;
	}
	inline void SetStartPoint(const sf::Vector2f& start)
	{
		start_end_points[0].position = start;
	}
	inline void SetEndPoint(const sf::Vector2f& end)
	{
		start_end_points[1].position = end;
	}
	inline void SetColor(const sf::Color& col)
	{
		for (int i = 0; i < start_end_points.getVertexCount(); ++i) start_end_points[i].color = col;
	}
	inline sf::Vector2f GetStartPoint() const
	{
		return start_end_points[0].position;
	}
	inline sf::Vector2f GetEndPoint() const
	{
		return start_end_points[1].position;
	}
	inline sf::Vector2f GetGoalPoint() const
	{
		return goal_point;
	}
	void Render(sf::RenderTarget& target) const
	{
		target.draw(start_end_points);
	}
	inline sf::Vector2f GetGradient() const
	{
		return start_end_points[1].position - start_end_points[0].position;
	}
};



class Triangle
{
private:
	sf::VertexArray points;
public:
	Triangle(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& p3)
	{
		points = sf::VertexArray(sf::PrimitiveType::TriangleStrip, 3);
		points[0] = sf::Vertex(p1);
		points[1] = sf::Vertex(p2);
		points[2] = sf::Vertex(p3);
	}
	void SetColor(rgb col)
	{
		SetColor(sf::Color(255 * col.r, 255 * col.g, 255 * col.b, 210));
	}
	void SetColor(const sf::Color& c)
	{
		for (std::size_t i = 0; i < 3; ++i)
		{
			points[i].color = c;
		}
	}
	void Render(sf::RenderTarget& target)
	{
		target.draw(points);
	}
};



class Raycaster
{
private:
	//The point from which the rays will be cast from.
	sf::Vector2f source;
	//Ray storage.
	std::vector<Line> rays;
	std::vector<Line> radial_rays;
	mutable bool render_convex_hull{ true };
public:
	Raycaster(const sf::Vector2f& light_source = {})
		:source(light_source)
	{

	}
	std::vector<Triangle> convex_hull;
	inline void SetSource(const sf::Vector2f& m_source)
	{
		source = m_source;
		for (auto& ray : rays)
		{
			ray.SetStartPoint(source);
		}
	}
	void UpdateConvexHull()
	{
		convex_hull.clear();
		//Sort based on ray angle.
		std::sort(rays.begin(), rays.end(), [](const Line& ray1, const Line& ray2)
			{
				return Utility::CalculateAngle(ray1.GetGradient()) < Utility::CalculateAngle(ray2.GetGradient());
			});
		//Constructing triangles with adjacent rays and the source.
		auto ray_1 = rays.begin();
		auto ray_2 = ray_1 + 1;

		while (ray_2 != rays.end())
		{
			convex_hull.emplace_back(source, ray_1->GetEndPoint(), ray_2->GetEndPoint());
			++ray_1;
			++ray_2;
		}
		//For the final ray, we want to wrap back to the first ray, so as to complete the hull.
		convex_hull.emplace_back(source, ray_1->GetEndPoint(), rays.front().GetEndPoint());

		//Configuring the color of each triangle as a function of the number of triangles.
		for (int i = 0; i < convex_hull.size(); ++i)
		{
			hsv col;
			col.h = (360 / convex_hull.size()) * i;
			col.s = 0.9;
			col.v = 1;
			convex_hull[i].SetColor(hsv2rgb(col));
		}	
	}
	void Update(const std::vector<Line>& boundaries)
	{
		
		if (rays.empty()) return;
		//Loop through every single ray.
		for (Line& ray : rays)
		{
			sf::Vector2f ray_start = ray.GetStartPoint();
			sf::Vector2f ray_end = ray_start + Utility::Normalise(ray.GetGoalPoint() - ray_start);
			//The lowest T value will determine the length of the ray.
			float record_t = INT_MAX;
			//Checking to see whether a ray collides with any boundaries.
			for (const Line& boundary : boundaries)
			{
				sf::Vector2f boundary_start = boundary.GetStartPoint();
				sf::Vector2f boundary_end = boundary.GetEndPoint();
				float TN = (ray_start.x - boundary_start.x) * (boundary_start.y - boundary_end.y) - (ray_start.y - boundary_start.y) * (boundary_start.x - boundary_end.x);
				float TD = (ray_start.x - ray_end.x) * (boundary_start.y - boundary_end.y) - (ray_start.y - ray_end.y) * (boundary_start.x - boundary_end.x);

				float UN = ((ray_start.x - ray_end.x) * (ray_start.y - boundary_start.y) - (ray_start.y - ray_end.y) * (ray_start.x - boundary_start.x)) * -1;
				float UD = (ray_start.x - ray_end.x) * (boundary_start.y - boundary_end.y) - (ray_start.y - ray_end.y) * (boundary_start.x - boundary_end.x);

				float T = TN / TD;
				float U = UN / UD;
				//Parameters for valid collision.
				if (T > 0 && U > 0 && U < 1 && T < record_t) record_t = T;
			}
			//Adjust the length of the ray if there if it hit a boundary.
			if (static_cast<int>(record_t) != INT_MAX)
			{
				ray.SetEndPoint(sf::Vector2f{ ray_start.x + record_t * (ray_end.x - ray_start.x), ray_start.y + record_t * (ray_end.y - ray_start.y) });
			}
		
		}
		//Restitch the convex hull.
		UpdateConvexHull();
	
	}
	void Render(sf::RenderTarget& target) 
	{
		for (const auto& ray : rays)
		{
			ray.Render(target);
		}
		if (!render_convex_hull) return;
		for (auto& triangle : convex_hull)
		{
			triangle.Render(target);
		}
		
	}
	void RegisterBoundary(const Line& boundary)
	{
		//For every boundary vertex, we create two rays.
		//Each ray is offset by a very small amount from its original goal.
		sf::Vector2f boundary_unit_gradient = Utility::Normalise(boundary.GetEndPoint() - boundary.GetStartPoint());
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetStartPoint() + boundary_unit_gradient, sf::Color::White);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetStartPoint() - boundary_unit_gradient, sf::Color::White);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetEndPoint() + boundary_unit_gradient, sf::Color::White);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetEndPoint() - boundary_unit_gradient, sf::Color::White);
	}
	inline void ToggleConvexHull() const noexcept { (render_convex_hull) ? render_convex_hull = false : render_convex_hull = true; }

};
Line CreateLine(const sf::Vector2f& start, const sf::Vector2f& end, const sf::Vector2f& goal, const sf::Color& col)
{
	return Line(start, end, goal, col);
}
int main()
{
	//We store the boundaries here, and not in the raycasting class, so as to accomodate for multiple light points.
	std::vector<Line> boundaries;
	bool mouse_clicked{ false };
	sf::RenderWindow window(sf::VideoMode(WIN_WIDTH, WIN_HEIGHT), "Raycasting Simulation", sf::Style::Default);

	std::vector<Raycaster> light_sources;
	light_sources.emplace_back();
	//light_sources.emplace_back(sf::Vector2f{ 500,500 });

	//Initialising the side boundaries to limit ray length.
	{
		float sensitivity = 5;
		for (Raycaster& raycaster : light_sources)
		{
			raycaster.RegisterBoundary((CreateLine(sf::Vector2f{ -sensitivity,-sensitivity }, sf::Vector2f{ WIN_WIDTH + sensitivity, -sensitivity }, { 0,0 }, sf::Color::White)));
			raycaster.RegisterBoundary(CreateLine(sf::Vector2f{ -sensitivity,-sensitivity }, sf::Vector2f{ -sensitivity,WIN_HEIGHT + sensitivity }, { 0,0 }, sf::Color::White));
			raycaster.RegisterBoundary(CreateLine(sf::Vector2f{ -sensitivity,WIN_HEIGHT + sensitivity }, sf::Vector2f{ WIN_WIDTH + sensitivity,WIN_HEIGHT + sensitivity }, { 0,0 }, sf::Color::White));
			raycaster.RegisterBoundary(CreateLine(sf::Vector2f{ WIN_WIDTH + sensitivity,0 - sensitivity }, sf::Vector2f{ WIN_WIDTH + sensitivity,WIN_HEIGHT + sensitivity }, { 0,0 }, sf::Color::White));
		}
	}

	while (window.isOpen())
	{
		sf::Event evnt; 
		while (window.pollEvent(evnt))
		{
			switch (evnt.type)
			{
			case sf::Event::EventType::Closed:
			{
				window.close();
				break;
			}
			//Drawing boundaries
			case sf::Event::EventType::MouseButtonPressed:
			{
				(mouse_clicked) ? mouse_clicked = false : mouse_clicked = true;
				auto mouse_position = static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
				if (mouse_clicked) boundaries.emplace_back(mouse_position, mouse_position, sf::Vector2f{ 0,0 }, sf::Color::White);
				else
				{

					//After the second click, we set the end point of the boundary, and register it.
					boundaries.back().SetEndPoint(mouse_position);
					for (Raycaster& ray_caster : light_sources)
					{
						ray_caster.RegisterBoundary(boundaries.back());
					}
;				}
					
				break;
			}
			case sf::Event::EventType::MouseMoved:
			{
				//Light source follows mouse.
				
				light_sources.front().SetSource(static_cast<sf::Vector2f>(sf::Mouse::getPosition(window)));
				break;
			}
			case sf::Event::EventType::KeyPressed:
			{
				if (evnt.key.code == sf::Keyboard::Key::Space)
				{
					for (Raycaster& raycaster : light_sources)
					{
						raycaster.ToggleConvexHull();
					}
					
				}
			}

			}
		}
		for (Raycaster& raycaster : light_sources)
		{
			raycaster.Update(boundaries);
		}
		window.clear();
		for (Raycaster& raycaster : light_sources)
		{
			raycaster.Render(window);
		}
		for (auto& boundary : boundaries) boundary.Render(window);
		window.display();
	}
}