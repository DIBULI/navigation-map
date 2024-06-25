#ifndef GRID_HPP
#define GRID_HPP

enum GridState {
  OCCUPIED,
  UNOCCUPIED
};

class Grid {
public:
  Grid();
  ~Grid();

  GridState state;

  
};

#endif /* GRID_HPP */
