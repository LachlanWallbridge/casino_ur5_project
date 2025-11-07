# Player.msg

string player_id
int32 position  # Position can be 1, 2, or 3
bool bet_is_odd

# Bet fields
string[] bet_colors                # One or more chip colours placed by the player
float32[] bet_x                    # Centroids X (in px)
float32[] bet_y                    # Centroids Y (in px)
float32[] bet_diameter_px          # Corresponding chip diameters