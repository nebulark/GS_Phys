#include "stdafx.h"
#include <SFML/Graphics.hpp>
#include "reactphysics3d.h"
#include "Simulation.h"
#include <chrono>

int main()
{

	sf::RenderWindow window(sf::VideoMode(1200, 900), "SFML works!");
	auto view = window.getView();
	view.setCenter(sf::Vector2f(0.f, 0.f));
	window.setView(view);

	Simulation simulation;
	simulation.Init();

	using float_seconds = std::chrono::duration<float>;
	using clock = std::chrono::steady_clock;
	
	clock::time_point lastTime = clock::now();
	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}
		clock::time_point currentTime = clock::now();
		float_seconds deltaSeconds = currentTime - lastTime;
		lastTime = currentTime;
		simulation.Update(deltaSeconds.count());
		window.clear();
		simulation.Render(window);
		window.display();
	}

	return 0;
}