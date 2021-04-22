#include "keyboard.h"

keyboard::Keyboard::Keyboard( int repeat_delay, int repeat_interval )
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0) throw std::runtime_error("Could not init SDL");
  SDL_EnableKeyRepeat( repeat_delay, repeat_interval );
  SDL_WM_SetCaption("ROS keyboard input", NULL);
  window = SDL_SetVideoMode(100, 100, 0, 0);
  

}

keyboard::Keyboard::~Keyboard(void)
{
  SDL_FreeSurface(window);
  SDL_Quit();
}

bool keyboard::Keyboard::get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers)
{
  new_event = false;
  
  SDL_Event event;
  if (SDL_PollEvent(&event)) {
    switch(event.type) {
      case SDL_KEYUP:
        pressed = false;
        code = event.key.keysym.sym;
        modifiers = event.key.keysym.mod;
        new_event = true;
        SDL_FillRect(window, NULL,SDL_MapRGB(window->format, code, 0, 0));  
        SDL_Flip(window);   
      break;
      case SDL_KEYDOWN:
        pressed = true;
        code = event.key.keysym.sym;
        modifiers = event.key.keysym.mod;
        new_event = true;
        SDL_FillRect(window, NULL, SDL_MapRGB(window->format,code,0,255)); 
        SDL_Flip(window);

      break;
      case SDL_QUIT:
        return false;
      break;
    }
  }
  return true;
}

