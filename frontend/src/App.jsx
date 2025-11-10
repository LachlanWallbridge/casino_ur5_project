import React, { useEffect, useState } from 'react';
import CasinoDashboard from './CasinoDashboard';
import { subscribeToPlayers } from './ui_bridge';
import 'bootstrap/dist/css/bootstrap.min.css';

function App() {
    const [players, setPlayers] = useState([]);

    useEffect(() => {
        const unsubscribe = subscribeToPlayers(async (playersMsg) => {
            // Sort by position
            const sorted = playersMsg.sort((a, b) => a.position - b.position);

            // Fetch backend stats for each player
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

    return <CasinoDashboard players={players} />;
}

export default App;
