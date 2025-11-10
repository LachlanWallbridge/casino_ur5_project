import React from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';

function PlayerCard({ player }) {
    const hasBets = player.bet_colors && player.bet_colors.length > 0;

    return (
        <div className="card text-center shadow-lg border-0"
             style={{
                 width: '22rem',
                 borderRadius: '16px',
                 background: 'linear-gradient(145deg, #2b2b2b, #1c1c1c)',
                 color: '#f8f8f8',
                 boxShadow: '0 0 18px rgba(212, 175, 55, 0.4), inset 0 0 10px rgba(255, 215, 0, 0.1)',
                 transform: 'scale(1.05)',
                 transition: 'all 0.2s ease-in-out'
             }}>
            <div className="card-body py-4">
                <h4 className="card-title mb-3" style={{ color: '#ffd700', textShadow: '0 0 6px rgba(255,215,0,0.7)' }}>
                    🎲 Player {player.player_id}
                </h4>
                <p className="mb-2"><strong>Position:</strong> {player.position ?? '-'}</p>
                <p className="mb-2"><strong>Balance:</strong> ${player.balance ?? 0}</p>
                <p className="mb-3"><strong>Games:</strong> {player.games_played ?? 0} played, {player.games_won ?? 0} won</p>

                <hr style={{ borderTop: '1px solid rgba(255,255,255,0.1)' }} />

                <h6 className="text-uppercase text-warning mb-3">Bets</h6>

                {hasBets ? (
                    <div className="d-flex flex-wrap justify-content-center gap-3">
                        {player.bet_colors.map((color, idx) => (
                            <div key={idx} className="d-flex flex-column align-items-center">
                                <div
                                    className="rounded-circle"
                                    style={{
                                        width: '48px',
                                        height: '48px',
                                        background: color,
                                        border: '2px solid #fff',
                                        boxShadow: `0 0 10px ${color}, inset 0 0 4px rgba(255,255,255,0.4)`
                                    }}
                                ></div>
                                <small className="text-light mt-1" style={{ fontSize: '0.8em', opacity: 0.85 }}>
                                    {color.charAt(0).toUpperCase() + color.slice(1)}
                                </small>
                            </div>
                        ))}
                    </div>
                ) : (
                    <p className="text-muted fst-italic mb-0">No bets placed</p>
                )}
            </div>
        </div>
    );
}

export default PlayerCard;
