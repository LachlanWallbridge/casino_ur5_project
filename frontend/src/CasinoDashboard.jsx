import React from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';
import Board from './Board';
import PlayerCard from './PlayerCard';

function CasinoDashboard({
    players,
    onBetChange,
    refreshBackendStats,
    roundActive,
    onRoundStateChange
}) {
    return (
        <div
            className="container-fluid vh-100 d-flex flex-column p-0"
            style={{ background: '#1a1a1a' }}
        >
            {/* Header */}
            <div
                className="py-3 text-center"
                style={{
                    background: 'linear-gradient(90deg, #111, #222)',
                    color: '#ffd700',
                    fontFamily: "'Cinzel Decorative', serif",
                    letterSpacing: '2px',
                    textShadow: '0 0 6px rgba(255, 215, 0, 0.6)',
                    borderBottom: '2px solid #d4af37',
                }}
            >
                <h2 className="m-0">🎰 Casino Table Dashboard</h2>
            </div>

            {/* Main layout */}
            <div
                className="d-flex flex-grow-1 px-4"
                style={{
                    paddingTop: '10px',
                    paddingBottom: '10px',
                    background: '#111',
                }}
            >
                {/* Board (70%) */}
                <div
                    className="flex-grow-1 d-flex align-items-center justify-content-center"
                    style={{
                        flexBasis: '70%',
                        background: 'radial-gradient(circle at center, #006400 0%, #013220 100%)',
                        boxShadow: 'inset 0 0 80px rgba(0,0,0,0.6)',
                        borderRadius: '20px',
                        marginRight: '15px',
                    }}
                >
                    <Board
                        players={players}
                        refreshBackendStats={refreshBackendStats}
                        onRoundStateChange={onRoundStateChange}
                    />
                </div>

                {/* Players sidebar (30%) */}
                <div
                    className="d-flex flex-column justify-content-evenly align-items-center p-4"
                    style={{
                        flexBasis: '30%',
                        background: 'radial-gradient(circle, #111 0%, #000 100%)',
                        borderRadius: '20px',
                        color: '#f8f8f8',
                        borderLeft: '2px solid #d4af37',
                    }}
                >
                    {players && players.length > 0 ? (
                        players.map((p, idx) => (
                            <div
                                key={idx}
                                className="my-3"
                                style={{
                                    width: '100%',
                                    display: 'flex',
                                    justifyContent: 'center',
                                }}
                            >
                                <PlayerCard
                                    player={p}
                                    onBetChange={onBetChange}
                                    roundActive={roundActive}
                                />
                            </div>
                        ))
                    ) : (
                        <p className="text-light mt-5 fst-italic">Waiting for player data...</p>
                    )}
                </div>
            </div>
        </div>
    );
}

export default CasinoDashboard;
