import React from 'react';

function PlayerCard({ player }) {
    return (
        <div style={{
            border: '1px solid #333',
            borderRadius: '8px',
            padding: '10px',
            margin: '5px',
            width: '150px',
            background: '#fff'
        }}>
            <h3>Player {player.player_id}</h3>
            <p>Balance: {player.balance ?? 0}</p>
            <p>Games Played: {player.games_played ?? 0}</p>
            <p>Games Won: {player.games_won ?? 0}</p>
        </div>
    );
}

export default PlayerCard;
