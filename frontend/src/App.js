import React, { useEffect, useState } from 'react';
import Board from './Board';
import PlayerCard from './PlayerCard';
import { subscribeToPlayers } from './ui_bridge';

function App() {
    const [players, setPlayers] = useState([]);

    useEffect(() => {
        const unsubscribe = subscribeToPlayers(async (playersMsg) => {
            // Sort by position
            const sorted = playersMsg.sort((a, b) => a.position - b.position);

            // Fetch stats from backend for each player
            const playersWithStats = await Promise.all(
                sorted.map(async (p) => {
                    try {
                        const res = await fetch(`/players/${p.player_id}`);
                        const data = await res.json();
                        return { ...p, ...data };
                    } catch (err) {
                        console.error('Error fetching player', p.player_id, err);
                        return { ...p, balance: 0, games_played: 0, games_won: 0 };
                    }
                })
            );

            setPlayers(playersWithStats);
        });

        return () => unsubscribe();
    }, []);

    return (
        <div style={{ display: 'flex', flexDirection: 'column', height: '100vh', backgroundColor: 'lightblue' }}>
            <div style={{ flex: 3 }}>
                <Board />
            </div>
            <div style={{ flex: 1, display: 'flex', flexWrap: 'wrap', padding: '10px', background: '#f0f0f0' }}>
                {players.map(player => (
                    <PlayerCard key={player.player_id} player={player} />
                ))}
            </div>
        </div>
    );
}

export default App;
