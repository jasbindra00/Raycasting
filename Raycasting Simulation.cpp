#include <SFML/Graphics.hpp>
#include <vector>
#include <array>
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



class Line
{
private:

	sf::VertexArray start_end_points;
	sf::Vector2f goal_point;
	void SetStartEnd(const sf::Vector2f& start, const sf::Vector2f& end)
	{

	}
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

	Triangle(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& p3, const sf::Color& color)
	{
		points = sf::VertexArray(sf::PrimitiveType::TriangleStrip, 3);
		points[0] = sf::Vertex(p1, color);
		points[1] = sf::Vertex(p2, color);
		points[2] = sf::Vertex(p3, color);
	}
	void Render(sf::RenderTarget& target)
	{
		target.draw(points);
	}


};



class Raycasting
{
private:
	sf::Vector2f source;
	std::vector<Line> rays;
public:
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
		std::sort(rays.begin(), rays.end(), [](const Line& ray1, const Line& ray2)
			{
				//Ordered before if the angle is less than the next.
				return Utility::CalculateAngle(ray1.GetGradient()) > Utility::CalculateAngle(ray2.GetGradient());
			});
		auto ray_1 = rays.begin();
		auto ray_2 = ray_1 + 1;
		while (ray_2 != rays.end())
		{
			convex_hull.emplace_back(source, ray_1->GetEndPoint(), ray_2->GetEndPoint(), sf::Color::Red);
			++ray_1;
			++ray_2;
		}
		convex_hull.emplace_back(source, ray_1->GetEndPoint(), rays.front().GetEndPoint(), sf::Color::Red);
	
	}
	void Update(const std::vector<Line>& boundaries)
	{
		if (rays.empty()) return;
		//Loop through every single ray.
		for (Line& ray : rays)
		{
			sf::Vector2f ray_start = ray.GetStartPoint();
			sf::Vector2f ray_end = ray_start + Utility::Normalise(ray.GetGoalPoint() - ray_start);
			float record_t = INT_MAX;
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

				if (T > 0 && U > 0 && U < 1 && T < record_t) record_t = T;
			}

			if (static_cast<int>(record_t) != INT_MAX)
			{
				ray.SetEndPoint(sf::Vector2f{ ray_start.x + record_t * (ray_end.x - ray_start.x), ray_start.y + record_t * (ray_end.y - ray_start.y) });
			}
		
		}
		UpdateConvexHull();
	
	}
	void Render(sf::RenderTarget& target) 
	{
		for (const auto& ray : rays)
		{
			ray.Render(target);
		}
		for (auto& triangle : convex_hull)
		{
			triangle.Render(target);
		}
		
	}
	void RegisterBoundary(const Line& boundary)
	{
		//For every boundary vertex, we create two rays.
		sf::Vector2f boundary_unit_gradient = Utility::Normalise(boundary.GetEndPoint() - boundary.GetStartPoint());
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetStartPoint() + boundary_unit_gradient, sf::Color::Red);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetStartPoint() - boundary_unit_gradient, sf::Color::Red);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetEndPoint() + boundary_unit_gradient, sf::Color::Red);
		rays.emplace_back(source, sf::Vector2f{ 0,0 }, boundary.GetEndPoint() - boundary_unit_gradient, sf::Color::Red);
	}
};


int main()
{
	std::vector<Line> boundaries;
	bool click{ false };


	sf::RenderWindow window(sf::VideoMode(900, 900), "Raycasting Simulation", sf::Style::Default);
	Raycasting raycasting;
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
			case sf::Event::EventType::MouseButtonPressed:
			{
				//Toggle the click state.
				(click) ? click = false : click = true;
				auto mouse_position = static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
				if (click) boundaries.emplace_back(mouse_position, mouse_position, sf::Vector2f{ 0,0 }, sf::Color::White);
				else
				{
					boundaries.back().SetEndPoint(mouse_position);
					raycasting.RegisterBoundary(boundaries.back());
				}
					
				break;
			}
			case sf::Event::EventType::MouseMoved:
			{
				raycasting.SetSource(static_cast<sf::Vector2f>(sf::Mouse::getPosition(window)));
				break;
			}

			}
		}
		raycasting.Update(boundaries);
		window.clear();
		raycasting.Render(window);
		for (auto& boundary : boundaries) boundary.Render(window);
		window.display();
	}



}