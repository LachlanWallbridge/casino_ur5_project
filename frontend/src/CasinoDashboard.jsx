import React from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';
import Board from './Board';
import PlayerCard from './PlayerCard';

function CasinoDashboard({
    players,
    onBetChange,
    refreshBackendStats,
    roundActive,
    onRoundStateChange,
    splitRatio = 0.5 // 0.6 => 60% board, 40% players
}) {
    const bottomRatio = 1 - splitRatio;

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

            {/* MAIN CONTENT - must be allowed to shrink */}
            <div
                className="flex-grow-1 d-flex flex-column"
                style={{ minHeight: 0 }}             // ⬅️ important
            >
                {/* --- Game board section --- */}
                <div
                    className="d-flex align-items-center justify-content-center"
                    style={{
                        flex: `${splitRatio} 1 0%`,    // ⬅️ explicit flex for ratio
                        minHeight: 0,
                        background: 'radial-gradient(circle at center, #006400 0%, #013220 100%)',
                        boxShadow: 'inset 0 0 80px rgba(0,0,0,0.6)',
                        borderRadius: '20px',
                        margin: '15px',
                        overflow: 'hidden',
                    }}
                >
                    <Board
                        players={players}
                        refreshBackendStats={refreshBackendStats}
                        onRoundStateChange={onRoundStateChange}
                    />
                </div>

                {/* --- Players section --- */}
                <div
                    style={{
                        flex: `${bottomRatio} 1 0%`,   // ⬅️ same idea here
                        minHeight: 0,                  // ⬅️ allow shrink
                        background: 'radial-gradient(circle, #111 0%, #000 100%)',
                        borderTop: '2px solid #d4af37',
                        borderRadius: '20px',
                        margin: '0 15px 15px 15px',
                        padding: '20px',
                        overflow: 'hidden',            // viewport for inner scroller
                        display: 'flex',
                        flexDirection: 'column',
                    }}
                >
                    <div
                        className="d-flex justify-content-center align-items-start w-100"
                        style={{
                            gap: '24px',
                            flex: '1 1 auto',          // ⬅️ this div is the scroller & must flex
                            minHeight: 0,              // ⬅️ must be allowed to be shorter than content
                            overflowY: 'auto',
                            paddingRight: '8px',
                        }}
                    >
                        {players && players.length > 0 ? (
                            players.map((p, idx) => (
                                <div
                                    key={idx}
                                    style={{
                                        flex: '0 1 22rem',
                                        maxWidth: '22rem',
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
                            <p className="text-light fst-italic mt-3">
                                Waiting for player data…
                            </p>
                        )}
                    </div>
                </div>
            </div>
        </div>
    );
}

export default CasinoDashboard;
