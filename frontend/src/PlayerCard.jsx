import React, { useState, useEffect } from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';

// Chip values
const COLOR_VALUE = {
    red: 100,
    white: 10,
    blue: 20,
    green: 50,
};

function computeWagerFromColors(colors) {
    return (colors || [])
        .map((c) => (c || '').toLowerCase())
        .reduce((sum, c) => sum + (COLOR_VALUE[c] || 0), 0);
}

// Group duplicates by colour
function groupChips(colors) {
    const counts = {};
    (colors || []).forEach((c) => {
        const key = c.toLowerCase();
        counts[key] = (counts[key] || 0) + 1;
    });
    return counts; // e.g. { red: 2, blue: 1 }
}

function PlayerCard({ player, onBetChange, roundActive }){
    // Local parity UI state
    const [bet, setBet] = useState("none");
    useEffect(() => {
        setBet(player.bet_ui ?? "none");
    }, [player]);

    const chooseBet = (value) => {
        setBet(value);
        onBetChange && onBetChange(player.player_id, value);
    };

    const chipGroups = groupChips(player.bet_colors);
    const totalWager = computeWagerFromColors(player.bet_colors);

    return (
        <div
            className="card text-center shadow-lg border-0"
            style={{
                width: '22rem',
                borderRadius: '16px',
                background: 'linear-gradient(145deg, #2b2b2b, #1c1c1c)',
                color: '#f8f8f8',
                boxShadow: '0 0 18px rgba(212,175,55,0.4), inset 0 0 10px rgba(255,215,0,0.1)',
            }}
        >
            <div className="card-body py-4">

                {/* Header */}
                <h4 className="card-title mb-3" style={{ color: '#ffd700', textShadow: '0 0 6px rgba(255,215,0,0.7)' }}>
                    🎲 Player {player.player_id}
                </h4>

                <p className="mb-2"><strong>Position:</strong> {player.position ?? '-'}</p>
                <p className="mb-2"><strong>Balance:</strong> ${player.balance ?? 0}</p>
                <p className="mb-3"><strong>Games:</strong> {player.games_played ?? 0} played, {player.games_won ?? 0} won</p>

                <hr style={{ borderTop: '1px solid rgba(255,255,255,0.1)' }} />

                {/* ---- PARITY SELECTOR ---- */}
                <h6 className="text-uppercase text-warning mb-2">Bet Parity</h6>

                <div className="d-flex align-items-center justify-content-center gap-4 mb-3"
                    style={{ minHeight: '40px' }}>

                    {/* LEFT ARROW SLOT */}
                    <div style={{ width: '30px', textAlign: 'center' }}>
                        {bet !== "odd" ? (
                            <button
                                onClick={!roundActive ? () => chooseBet("odd") : undefined}
                                disabled={roundActive}
                                style={{
                                    background: 'none',
                                    border: 'none',
                                    color: '#ff4d4d',
                                    fontSize: '2rem',
                                    cursor: roundActive ? 'default' : 'pointer',
                                    opacity: roundActive ? 0.3 : 1,
                                    textShadow: '0 0 6px rgba(255,80,80,0.8)',
                                    lineHeight: '1em',
                                }}
                            >
                                ←
                            </button>
                        ) : (
                            <span style={{ visibility: 'hidden', fontSize: '2rem' }}>←</span>
                        )}
                    </div>

                    {/* CENTER LABEL */}
                    <span
                        style={{
                            fontSize: '1.4rem',
                            fontWeight: 'bold',
                            color:
                                bet === "none"
                                    ? '#ccc'
                                    : bet === "odd"
                                    ? '#ff4d4d'
                                    : '#4da6ff',
                            textShadow:
                                bet === "odd"
                                    ? '0 0 6px rgba(255,80,80,0.7)'
                                    : bet === "even"
                                    ? '0 0 6px rgba(80,160,255,0.7)'
                                    : '0 0 6px rgba(255,255,255,0.3)',
                            width: '90px',
                            textAlign: 'center',
                        }}
                    >
                        {bet === "none"
                            ? 'No Bet'
                            : bet === "odd"
                            ? 'Odd'
                            : 'Even'}
                    </span>

                    {/* RIGHT ARROW SLOT */}
                    <div style={{ width: '30px', textAlign: 'center' }}>
                        {bet !== "even" ? (
                            <button
                                onClick={!roundActive ? () => chooseBet("even") : undefined}
                                disabled={roundActive}
                                style={{
                                    background: 'none',
                                    border: 'none',
                                    color: '#4da6ff',
                                    fontSize: '2rem',
                                    cursor: roundActive ? 'default' : 'pointer',
                                    opacity: roundActive ? 0.3 : 1,
                                    textShadow: '0 0 6px rgba(80,160,255,0.8)',
                                    lineHeight: '1em',
                                }}
                            >
                                →
                            </button>
                        ) : (
                            <span style={{ visibility: 'hidden', fontSize: '2rem' }}>→</span>
                        )}
                    </div>

                </div>




                {/* ---- CHIP DISPLAY ---- */}
                <h6 className="text-uppercase text-warning mb-3">Chips</h6>

                {Object.keys(chipGroups).length > 0 ? (
                    <div className="d-flex flex-wrap justify-content-center gap-3">

                        {Object.entries(chipGroups)
                            .sort(([colorA], [colorB]) => COLOR_VALUE[colorA] - COLOR_VALUE[colorB])
                            .map(([color, count], idx) => (
                            <div key={idx} className="d-flex flex-column align-items-center">

                                {/* Chip circle */}
                                <div
                                    className="d-flex align-items-center justify-content-center"
                                    style={{
                                        width: '55px',
                                        height: '55px',
                                        borderRadius: '50%',
                                        background: color,
                                        border: '3px solid white',
                                        color: '#000',
                                        fontWeight: 800,
                                        fontSize: '1.1rem',
                                        textShadow: `
                                            -1px -1px 0 #fff,
                                            1px -1px 0 #fff,
                                            -1px  1px 0 #fff,
                                            1px  1px 0 #fff
                                        `,  
                                        boxShadow: `0 0 12px ${color}, inset 0 0 4px rgba(255,255,255,0.5)`,
                                    }}
                                >
                                    {COLOR_VALUE[color]}
                                </div>

                                {/* If multiple chips → show xN */}
                                {count > 1 && (
                                    <small className="text-warning mt-1" style={{ fontSize: '0.8em' }}>
                                        ×{count}
                                    </small>
                                )}

                                {/* Color label */}
                                <small className="text-light mt-1" style={{ opacity: 0.85 }}>
                                    {color.charAt(0).toUpperCase() + color.slice(1)}
                                </small>
                            </div>
                        ))}

                    </div>
                ) : (
                    <p className="text-muted fst-italic mb-0">No chips placed</p>
                )}

                {/* ---- TOTAL WAGER ---- */}
                {totalWager > 0 && (
                    <p className="mt-3" style={{ fontSize: '1.1rem', color: '#ffd700' }}>
                        Total Bet: <strong>${totalWager}</strong>
                    </p>
                )}
            </div>
        </div>
    );
}

export default PlayerCard;
