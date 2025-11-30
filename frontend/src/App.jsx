import React, { useEffect, useState, useRef } from 'react';
import CasinoDashboard from './CasinoDashboard';
import { subscribeToPlayers } from './ui_bridge';
import 'bootstrap/dist/css/bootstrap.min.css';

function App() {
    const [players, setPlayers] = useState([]);
    const playersRef = useRef([]);
    
    const [bets, setBets] = useState({});
    const betsRef = useRef({});

    const [roundActive, setRoundActive] = useState(false);

    useEffect(() => {
        playersRef.current = players;
    }, [players]);

    useEffect(() => {
        betsRef.current = bets;
    }, [bets]);

    const handleBetChange = (playerId, value) => {
        setBets(prev => ({
            ...prev,
            [playerId]: value
        }));
    };

    useEffect(() => {
        let backendCache = {}; // { player_id: {balance, games...} }

        const unsubscribe = subscribeToPlayers(async (playersMsg) => {
            // Sort by position
            const sorted = playersMsg.sort((a, b) => a.position - b.position);

            // Determine which players need backend fetch
            const fetchList = sorted.filter(
                p => !backendCache[p.player_id] // not yet cached
            );

            // Fetch only new players
            if (fetchList.length > 0) {
                await Promise.all(
                    fetchList.map(async (p) => {
                        try {
                            const res = await fetch(`/players/${p.player_id}`);
                            const data = await res.json();
                            backendCache[p.player_id] = data; // save in cache
                            return null;
                        } catch (err) {
                            console.error('Error fetching player', p.player_id, err);
                            backendCache[p.player_id] = {
                                balance: 0, games_played: 0, games_won: 0
                            };
                            return null;
                        }
                    })
                );
            }

            const merged = sorted.map(p => {
                return {
                    ...p,                                // ROS live fields
                    ...(backendCache[p.player_id] || {}),// backend stats
                    bet_ui: betsRef.current[p.player_id] ?? "none", // current bet UI selection
                };
            });

            setPlayers(merged);
        });

        return () => unsubscribe();
    }, []);

    const refreshBackendStats = async () => {
        const updated = await Promise.all(
            players.map(async (p) => {
                try {
                    const res = await fetch(`/players/${p.player_id}`);
                    const data = await res.json();
                    return { ...p, ...data };
                } catch {
                    return p;
                }
            })
        );
        setPlayers(updated);
    };

    return <CasinoDashboard
        players={players}
        onBetChange={handleBetChange}
        refreshBackendStats={refreshBackendStats}
        roundActive={roundActive}
        onRoundStateChange={setRoundActive}
    />
}

export default App;
